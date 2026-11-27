//! Quality refinement (the refine.rs rows of docs/function-mapping.md):
//! tallyencs, precisionerror, splitencsegs, tallyfaces, splittriangle,
//! enforcequality, and highorder. C: the Mesh quality maintenance section
//! of triangle.c (tallyencs through enforcequality) plus highorder, which
//! moves from just before the Array I/O banner (triangle.c:7624; the
//! banner sits at triangle.c:7687) for cohesion with its only callers,
//! the quality stage of the api. The aCute call site findNewSPLocation
//! (triangle.c:7437) is live since stage-1 acute C (#43): it routes
//! through the family's D26 seam (tuple interpolants, the Oom funnel).
//! The aCute scratch-pool init (triangle.c:7536-7540) stays a
//! documented no-op: the pool is eager since the D23 dissolution
//! (Mesh::new always carries it). Assignment:
//! docs/function-mapping.md.

use std::cmp::Ordering;

use crate::error::{Error, SegmentRecoveryKind};
use crate::handles::{decode, sdecode, Osub, Otri, DUMMY_SUB, DUMMY_TRI, NULL};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::{tri_counterclockwise, tri_findcircumcenter};
use crate::quality::{checkseg4encroach, dequeuebadtriang, enqueuebadtriang, testtriangle};
use crate::slots::{FREEVERTEX, SEGMENTVERTEX};
use crate::transform::{deletevertex, insertvertex, undovertex, InsertVertexResult};
use crate::util::vertex_coords;

/// C: tallyencs (triangle.c:7129-7142). Traverse the entire list of
/// subsegments, and check each to see if it is encroached. If so, add it
/// to the list (checkseg4encroach does the adding).
pub(crate) fn tallyencs(m: &mut Mesh, b: &Behavior) -> Result<(), Oom> {
    let mut t = m.subsegs.traversal();
    // C: subsegloop.ssorient = 0; subsegloop.ss = subsegtraverse(m)
    // (triangle.c:7134-7136); the end-of-pool NULL is the None here.
    while let Some(ss) = m.subsegtraverse(&mut t) {
        let subsegloop = Osub { ss, ssorient: 0 };
        // If the segment is encroached, add it to the list
        // (triangle.c:7138). C assigns the mask to the unread local
        // 'dummy' (triangle.c:7139); the port drops the dead store (D17).
        checkseg4encroach(m, b, subsegloop)?;
    }
    Ok(())
}

/// C: precisionerror (triangle.c:7154-7163). Print an error message for
/// precision problems; the two stdout lines are C's printfs verbatim.
/// The SINGLE block (triangle.c:7158-7162) compiles out of the pinned
/// double-precision oracle build, exactly like the port's f64.
///
/// The only C call site sits in splittriangle's _DEBUG tail
/// (triangle.c:7510), which the pinned oracle build compiles out; the
/// port mirrors the compiled-out call (the tail keeps its branch shape),
/// so the 1:1 body is ahead-of-use until the CLI stdout wiring calls it.
#[allow(dead_code)]
// C-parity: the only call site (triangle.c:7510) is
// _DEBUG-gated and compiled out of the pinned oracle
// build; the verbatim body awaits the CLI wiring.
#[allow(clippy::print_with_newline)] // C-parity: the two printfs verbatim (triangle.c:7156-7157)
pub(crate) fn precisionerror() {
    print!("Try increasing the area criterion and/or reducing the minimum\n");
    print!("  allowable angle so that tiny triangles are not created.\n");
}

/// C: splitencsegs (triangle.c:7183-7371). Split all the encroached
/// subsegments: each encroached subsegment is repaired by splitting it -
/// inserting a vertex at or near its midpoint - and newly inserted
/// vertices may encroach upon other subsegments, which are also
/// repaired. 'triflaws' specifies whether to take note of new bad
/// triangles that result from inserting vertices to repair encroached
/// subsegments. C's 'int *status' out-param is dissolved (rung 2,
/// D8): the negative writes raise Error::SegmentRecovery in place -
/// TRI_SEG_SPLIT (triangle.c:7342, 7355) and TRI_CYCLE
/// (triangle.c:7349).
pub(crate) fn splitencsegs(m: &mut Mesh, b: &Behavior, triflaws: bool) -> Result<(), Error> {
    // Note that steinerleft == -1 if an unlimited number of Steiner
    // points is allowed (triangle.c:7202-7203).
    while (m.badsubsegs.live_count() > 0) && (m.steinerleft != 0) {
        let mut t = m.badsubsegs.traversal();
        let mut encloop = m.badsubsegtraverse(&mut t);
        // C: while ((encloop != NULL) && (m->steinerleft != 0))
        // (triangle.c:7207); the while-let's None is the NULL, and the
        // conjunct order is C's.
        while let Some(encloop_row) = encloop {
            if m.steinerleft == 0 {
                break;
            }
            let mut currentenc = sdecode(m.bad_sub_seg_enc(encloop_row));
            let eorg = m.sorg(currentenc);
            let edest = m.sdest(currentenc);
            // Make sure that this segment is still the same segment it
            // was when it was determined to be encroached. If the
            // segment was enqueued multiple times (because several newly
            // inserted vertices encroached it), it may have already been
            // split (triangle.c:7211-7214).
            if !m.deadsubseg(currentenc.ss)
                && eorg == m.bad_sub_seg_org(encloop_row)
                && edest == m.bad_sub_seg_dest(encloop_row)
            {
                // To decide where to split a segment, we need to know if
                // the segment shares an endpoint with an adjacent
                // segment. (The concentric-circles rationale of C's
                // comment block, triangle.c:7217-7230.)

                // Is the origin shared with another segment?
                // (triangle.c:7232)
                let mut enctri = m.stpivot(currentenc);
                let mut testtri = enctri.lnext();
                let mut testsh = m.tspivot(testtri);
                let mut acuteorg = testsh.ss != DUMMY_SUB;
                // Is the destination shared with another segment?
                // (triangle.c:7237)
                testtri.lnextself();
                testsh = m.tspivot(testtri);
                let mut acutedest = testsh.ss != DUMMY_SUB;

                // If we're using Chew's algorithm (rather than
                // Ruppert's) to define encroachment, delete free
                // vertices from the subsegment's diametral circle
                // (triangle.c:7242-7244).
                if !b.conformdel && !acuteorg && !acutedest {
                    let mut eapex = m.apex(enctri);
                    while m.vx_type(eapex) == FREEVERTEX
                        && (m.vx_x(eorg) - m.vx_x(eapex)) * (m.vx_x(edest) - m.vx_x(eapex))
                            + (m.vx_y(eorg) - m.vx_y(eapex)) * (m.vx_y(edest) - m.vx_y(eapex))
                            < 0.0
                    {
                        deletevertex(m, b, &mut testtri)?;
                        enctri = m.stpivot(currentenc);
                        eapex = m.apex(enctri);
                        testtri = enctri.lprev();
                    }
                }

                // Now, check the other side of the segment, if there's a
                // triangle there (triangle.c:7257-7258).
                testtri = m.sym(enctri);
                if testtri.tri != DUMMY_TRI {
                    // Is the destination shared with another segment?
                    // (triangle.c:7261)
                    testtri.lnextself();
                    testsh = m.tspivot(testtri);
                    let acutedest2 = testsh.ss != DUMMY_SUB;
                    acutedest = acutedest || acutedest2;
                    // Is the origin shared with another segment?
                    // (triangle.c:7266)
                    testtri.lnextself();
                    testsh = m.tspivot(testtri);
                    let acuteorg2 = testsh.ss != DUMMY_SUB;
                    acuteorg = acuteorg || acuteorg2;

                    // Delete free vertices from the subsegment's
                    // diametral circle (triangle.c:7272).
                    if !b.conformdel && !acuteorg2 && !acutedest2 {
                        let mut eapex = m.org(testtri);
                        while m.vx_type(eapex) == FREEVERTEX
                            && (m.vx_x(eorg) - m.vx_x(eapex)) * (m.vx_x(edest) - m.vx_x(eapex))
                                + (m.vx_y(eorg) - m.vx_y(eapex)) * (m.vx_y(edest) - m.vx_y(eapex))
                                < 0.0
                        {
                            deletevertex(m, b, &mut testtri)?;
                            testtri = m.sym(enctri);
                            eapex = m.apex(testtri);
                            testtri.lprevself();
                        }
                    }
                }

                // Use the concentric circles if exactly one endpoint is
                // shared with another adjacent segment
                // (triangle.c:7286-7287).
                let mut split: f64;
                if acuteorg || acutedest {
                    let segmentlength = ((m.vx_x(edest) - m.vx_x(eorg))
                        * (m.vx_x(edest) - m.vx_x(eorg))
                        + (m.vx_y(edest) - m.vx_y(eorg)) * (m.vx_y(edest) - m.vx_y(eorg)))
                    .sqrt();
                    // Find the power of two that most evenly splits the
                    // segment. The worst case is a 2:1 ratio between
                    // subsegment lengths (triangle.c:7291-7292).
                    let mut nearestpoweroftwo = 1.0;
                    while segmentlength > 3.0 * nearestpoweroftwo {
                        nearestpoweroftwo *= 2.0;
                    }
                    while segmentlength < 1.5 * nearestpoweroftwo {
                        nearestpoweroftwo *= 0.5;
                    }
                    // Where do we split the segment? (triangle.c:7300)
                    split = nearestpoweroftwo / segmentlength;
                    if acutedest {
                        split = 1.0 - split;
                    }
                } else {
                    // If we're not worried about adjacent segments,
                    // split this segment in the middle
                    // (triangle.c:7306-7307).
                    split = 0.5;
                }

                // Create the new vertex (triangle.c:7311).
                let newvertex = m.vx_alloc()?;
                // Interpolate its coordinate and attributes
                // (triangle.c:7313).
                for i in 0..(2 + m.nextras) as usize {
                    m.vx_set_real(
                        newvertex,
                        i,
                        m.vx_real(eorg, i) + split * (m.vx_real(edest, i) - m.vx_real(eorg, i)),
                    );
                }

                if !b.noexact {
                    // Roundoff in the above calculation may yield a
                    // 'newvertex' that is not precisely collinear with
                    // 'eorg' and 'edest'. Improve collinearity by one
                    // step of iterative refinement
                    // (triangle.c:7319-7321).
                    let mut multiplier = tri_counterclockwise(
                        m,
                        b,
                        vertex_coords(m, eorg),
                        vertex_coords(m, edest),
                        vertex_coords(m, newvertex),
                    );
                    let divisor = (m.vx_x(eorg) - m.vx_x(edest)) * (m.vx_x(eorg) - m.vx_x(edest))
                        + (m.vx_y(eorg) - m.vx_y(edest)) * (m.vx_y(eorg) - m.vx_y(edest));
                    if (multiplier != 0.0) && (divisor != 0.0) {
                        multiplier /= divisor;
                        // Watch out for NANs (triangle.c:7327).
                        #[allow(clippy::eq_op)] // C-parity: triangle.c:7328
                        // tests NaN by self-equality,
                        // verbatim.
                        if multiplier == multiplier {
                            m.set_vx_x(
                                newvertex,
                                m.vx_x(newvertex) + multiplier * (m.vx_y(edest) - m.vx_y(eorg)),
                            );
                            m.set_vx_y(
                                newvertex,
                                m.vx_y(newvertex) + multiplier * (m.vx_x(eorg) - m.vx_x(edest)),
                            );
                        }
                    }
                }

                m.vx_set_mark(newvertex, m.sub_mark(currentenc.ss));
                m.vx_set_type(newvertex, SEGMENTVERTEX);
                // Check whether the new vertex lies on an endpoint
                // (triangle.c:7337).
                if (m.vx_x(newvertex) == m.vx_x(eorg) && m.vx_y(newvertex) == m.vx_y(eorg))
                    || (m.vx_x(newvertex) == m.vx_x(edest) && m.vx_y(newvertex) == m.vx_y(edest))
                {
                    // C: TODO: Precision error: splitencsegs() - Ran out
                    // of precision at (newvertex)
                    // (triangle.c:7340-7341).
                    return Err(Error::SegmentRecovery(SegmentRecoveryKind::Split));
                }
                // Insert the splitting vertex. This should always
                // succeed (triangle.c:7345).
                let success = insertvertex(
                    m,
                    b,
                    newvertex,
                    &mut enctri,
                    Some(&mut currentenc),
                    true,
                    triflaws,
                    false,
                )?;
                if m.cyclefail {
                    return Err(Error::SegmentRecovery(SegmentRecoveryKind::Cycle));
                }
                if (success != InsertVertexResult::SuccessfulVertex)
                    && (success != InsertVertexResult::EncroachingVertex)
                {
                    // C: TODO: Internal error: splitencsegs() - Failure
                    // to split a segment (triangle.c:7353-7354).
                    return Err(Error::SegmentRecovery(SegmentRecoveryKind::Split));
                }
                if m.steinerleft > 0 {
                    m.steinerleft -= 1;
                }
                // Check the two new subsegments to see if they're
                // encroached (triangle.c:7361). C stores both masks into
                // the unread local 'dummy' (triangle.c:7362, 7364); the
                // port drops the dead stores (D17).
                checkseg4encroach(m, b, currentenc)?;
                m.snextself(&mut currentenc);
                checkseg4encroach(m, b, currentenc)?;
            }

            m.badsubsegdealloc(encloop_row);
            encloop = m.badsubsegtraverse(&mut t);
        }
    }
    Ok(())
}

/// C: tallyfaces (triangle.c:7383-7395). Test every triangle in the mesh
/// for quality measures; if the triangle is bad, testtriangle enqueues
/// it.
pub(crate) fn tallyfaces(m: &mut Mesh, b: &Behavior) -> Result<(), Oom> {
    let mut t = m.triangles.traversal();
    // C: triangleloop.orient = 0; triangleloop.tri = triangletraverse(m)
    // (triangle.c:7387-7389).
    while let Some(tri) = m.triangletraverse(&mut t) {
        let triangleloop = Otri { tri, orient: 0 };
        // If the triangle is bad, enqueue it (triangle.c:7391).
        testtriangle(m, b, triangleloop)?;
    }
    Ok(())
}

/// C: splittriangle (triangle.c:7409-7514). Create a new vertex at a bad
/// triangle's circumcenter (or, under the aCute build without an area
/// constraint, at findNewSPLocation's pick), insert it, and maintain the
/// Delaunay property. C is void; the port's Err carries the pool OOM
/// bridge and the findNewSPLocation call's Oom through From<Oom> for
/// Error (D26's rung-2 shape; D16, superseded by stage-1 acute C #43).
pub(crate) fn splittriangle(m: &mut Mesh, b: &Behavior, badtri: u32) -> Result<(), Error> {
    let mut badotri = decode(m.bad_tri_poortri(badtri));
    let borg = m.org(badotri);
    let bdest = m.dest(badotri);
    let bapex = m.apex(badotri);
    // Make sure that this triangle is still the same triangle it was
    // when it was tested and determined to be of bad quality.
    // Subsequent transformations may have made it a different triangle
    // (triangle.c:7423-7425).
    if !m.deadtri(badotri.tri)
        && borg == m.bad_tri_org(badtri)
        && bdest == m.bad_tri_dest(badtri)
        && bapex == m.bad_tri_apex(badtri)
    {
        let mut errorflag = 0;
        // Create a new vertex at the triangle's circumcenter
        // (triangle.c:7430).
        let newvertex = m.vx_alloc().map_err(Error::from)?;
        // C: #ifndef NO_ACUTE (triangle.c:7432-7441), mirrored as the
        // crate's 'acute' feature - the inverted compile switch,
        // exactly like quality.rs's testtriangle maxangle block.
        let (xi, eta): (f64, f64);
        #[cfg(feature = "acute")]
        {
            if b.fixedarea || b.vararea || b.usertest {
                // C: TODO: Acute fails for certain area constraints
                // (triangle.c:7434).
                let (x, y, xi_v, eta_v) = tri_findcircumcenter(
                    m,
                    b,
                    vertex_coords(m, borg),
                    vertex_coords(m, bdest),
                    vertex_coords(m, bapex),
                    true,
                );
                m.set_vx_x(newvertex, x);
                m.set_vx_y(newvertex, y);
                xi = xi_v;
                eta = eta_v;
            } else {
                // C: findNewSPLocation(m, b, borg, bdest, bapex,
                // newvertex, &xi, &eta, 1, badotri) (triangle.c:7437):
                // the row stays allocated through the call exactly as
                // C keeps it, the interpolants return as the tuple
                // (D26), and the Oom of the deletevertex inside
                // routes through From<Oom> for Error (D26's rung-2
                // shape).
                let (xi_v, eta_v) = crate::acute::findNewSPLocation(
                    m, b, borg, bdest, bapex, newvertex, true, badotri,
                )
                .map_err(Error::from)?;
                xi = xi_v;
                eta = eta_v;
            }
        }
        #[cfg(not(feature = "acute"))]
        {
            // C: tri_findcircumcenter (triangle.c:7440).
            let (x, y, xi_v, eta_v) = tri_findcircumcenter(
                m,
                b,
                vertex_coords(m, borg),
                vertex_coords(m, bdest),
                vertex_coords(m, bapex),
                true,
            );
            m.set_vx_x(newvertex, x);
            m.set_vx_y(newvertex, y);
            xi = xi_v;
            eta = eta_v;
        }

        // Check whether the new vertex lies on a triangle vertex, or
        // could not be placed at all: a triangle with no area has no
        // circumcenter, and the one computed for it is not finite
        // (triangle.c:7443-7445).
        if !m.vx_x(newvertex).is_finite()
            || !m.vx_y(newvertex).is_finite()
            || (m.vx_x(newvertex) == m.vx_x(borg) && m.vx_y(newvertex) == m.vx_y(borg))
            || (m.vx_x(newvertex) == m.vx_x(bdest) && m.vx_y(newvertex) == m.vx_y(bdest))
            || (m.vx_x(newvertex) == m.vx_x(bapex) && m.vx_y(newvertex) == m.vx_y(bapex))
        {
            // C: the _DEBUG warning printf (triangle.c:7450-7454)
            // compiles out of the pinned oracle build.
            errorflag = 1;
            m.vx_dealloc(newvertex);
        } else {
            // Interpolation of vertex attributes is done in insertvertex
            // method (triangle.c:7458).

            // The new vertex must be in the interior, and therefore is a
            // free vertex with a marker of zero (triangle.c:7460-7461).
            m.vx_set_mark(newvertex, 0);
            m.vx_set_type(newvertex, FREEVERTEX);

            // Ensure that the handle 'badotri' does not represent the
            // longest edge of the triangle. This ensures that the
            // circumcenter must fall to the left of this edge, so point
            // location will work. (If the angle org-apex-dest exceeds 90
            // degrees, then the circumcenter lies outside the org-dest
            // edge, and eta is negative. Roundoff error might prevent
            // eta from being negative when it should be, so I test eta
            // against xi.) (triangle.c:7465-7471)
            if eta < xi {
                badotri.lprevself();
            }

            // Insert the circumcenter, searching from the edge of the
            // triangle, and maintain the Delaunay property of the
            // triangulation (triangle.c:7476-7477).
            let success = insertvertex(m, b, newvertex, &mut badotri, None, true, true, true)
                .map_err(Error::from)?;
            if success == InsertVertexResult::SuccessfulVertex {
                if m.steinerleft > 0 {
                    m.steinerleft -= 1;
                }
            } else if success == InsertVertexResult::EncroachingVertex {
                // If the newly inserted vertex encroaches upon a
                // subsegment, delete the new vertex
                // (triangle.c:7485-7486).
                undovertex(m, b);
                m.vx_dealloc(newvertex);
            } else if success == InsertVertexResult::ViolatingVertex {
                // Failed to insert the new vertex, but some subsegment
                // was marked as being encroached
                // (triangle.c:7490-7491).
                m.vx_dealloc(newvertex);
            } else {
                // Couldn't insert the new vertex because a vertex is
                // already there (triangle.c:7493-7494). The _DEBUG
                // warning printf (triangle.c:7495-7498) compiles out.
                errorflag = 1;
                m.vx_dealloc(newvertex);
            }
        }
        if errorflag != 0 {
            // C: the _DEBUG printf block and the precisionerror() call
            // (triangle.c:7505-7511) compile out of the pinned oracle
            // build; the port mirrors the compiled-out branch shape.
        }
    }
    Ok(())
}

/// C: enforcequality (triangle.c:7527-7610). Remove all the encroached
/// subsegments and bad triangles from the triangulation. C's 'int
/// *status' out-param is dissolved (rung 2, D8): the TRI_CYCLE write
/// (triangle.c:7574) raises Error::SegmentRecovery in place. The
/// three poolinit resets (triangle.c:7533-7534, 7555-7556, 7565-7566)
/// mirror as the slots.rs restarts: the pools take their geometry
/// eagerly at construction (ADR 0003 section 4), and restart reproduces
/// poolinit's reset of the allocation state - the timing no allocation,
/// id, reuse, or traversal can observe, and what a re-refinement must
/// see again.
pub(crate) fn enforcequality(m: &mut Mesh, b: &Behavior) -> Result<(), Error> {
    // Initialize the pool of encroached subsegments
    // (triangle.c:7532).
    m.bad_sub_seg_restart();

    // C: #ifndef NO_ACUTE (triangle.c:7536-7540): the aCute scratch
    // pool, initialized by acutepool_init(20, &m->acute_mem) at
    // triangle.c:7538. The port does not re-implement this site: the
    // D23 dissolution made the pool an eager, always-present part of
    // Mesh::new (the triangle.c:1507 mapping), so the init is a
    // documented no-op here (D16, superseded by stage-1 acute C #43;
    // its only consumer, findNewSPLocation, owns its scratch per
    // call).

    // Test all segments to see if they're encroached
    // (triangle.c:7542).
    tallyencs(m, b)?;

    // Fix encroached subsegments without noting bad triangles
    // (triangle.c:7545).
    splitencsegs(m, b, false)?;

    // At this point, if we haven't run out of Steiner points, the
    // triangulation should be (conforming) Delaunay
    // (triangle.c:7549-7550).

    // Next, we worry about enforcing triangle quality
    // (triangle.c:7552).
    if (b.minangle > 0.0) || b.vararea || b.fixedarea || b.usertest {
        // Initialize the pool of bad triangles (triangle.c:7554).
        m.bad_tri_restart();
        // Initialize the queues of bad triangles
        // (triangle.c:7557-7561), the construction-pinned queue state
        // of mesh.rs re-initialized verbatim.
        for slot in m.queuefront.iter_mut() {
            *slot = NULL;
        }
        m.firstnonemptyq = -1;
        // Test all triangles to see if they're bad (triangle.c:7562).
        tallyfaces(m, b)?;
        // Initialize the pool of recently flipped triangles
        // (triangle.c:7564).
        m.flip_restart();
        m.checkquality = true;

        while (m.badtriangles.live_count() > 0) && (m.steinerleft != 0) {
            // Fix one bad triangle by inserting a vertex at its
            // circumcenter (triangle.c:7570).
            let badtri = match dequeuebadtriang(m) {
                Some(row) => row,
                // C's dequeue cannot return NULL under the loop guard
                // (every live row sits in a queue); the None arm mirrors
                // the impossible NULL.
                None => break,
            };
            splittriangle(m, b, badtri)?;
            if m.cyclefail {
                return Err(Error::SegmentRecovery(SegmentRecoveryKind::Cycle));
            }
            if m.badsubsegs.live_count() > 0 {
                // Put bad triangle back in queue for another try later
                // (triangle.c:7578).
                enqueuebadtriang(m, b, badtri);
                // Fix any encroached subsegments that resulted. Record
                // any new bad triangles that result
                // (triangle.c:7580-7581).
                splitencsegs(m, b, true)?;
            } else {
                // Return the bad triangle to the pool
                // (triangle.c:7585).
                m.bad_tri_dealloc(badtri);
            }
        }
    }
    // At this point, if the "-D" switch was selected and we haven't run
    // out of Steiner points, the triangulation should be (conforming)
    // Delaunay and have no low-quality triangles
    // (triangle.c:7590-7592).

    // Might we have run out of Steiner points too soon?
    // (triangle.c:7594) - the _DEBUG warning block
    // (triangle.c:7595-7609) compiles out of the pinned oracle build.
    Ok(())
}

/// C: highorder (triangle.c:7624-7685). Create extra nodes for quadratic
/// subparametric elements: loop over all triangles, look at the three
/// edges of each, and operate on an edge only if there isn't another
/// triangle adjacent to the edge, or if the current triangle has a
/// smaller pointer than its neighbor, so each edge is considered only
/// once (triangle.c:7643-7648). The raw pointer compare
/// (triangle.c:7653) mirrors as the oracle address order (ADR 0003
/// section 5's output-visible sites). Moved from just before C's Array
/// I/O banner for cohesion with the quality stage that calls it.
#[allow(dead_code)] // C-parity: the only call site is triangle_mesh_create
                    // (triangle_api.c:256), which awaits the ioarrays phase;
                    // the 1:1 body is ahead-of-use until that shell lands.
pub(crate) fn highorder(m: &mut Mesh, b: &Behavior) -> Result<(), Oom> {
    // The following line ensures that dead items in the pool of nodes
    // cannot be allocated for the extra nodes associated with high order
    // elements. This ensures that the primary nodes (at the corners of
    // elements) will occur earlier in the output files, and have lower
    // indices, than the extra nodes (triangle.c:7634-7638).
    m.vertices.clear_dead_stack();

    let mut t = m.triangles.traversal();
    let mut triangleloop = m.triangletraverse(&mut t);
    while let Some(tri) = triangleloop {
        for orient in 0..3u8 {
            let tri_edge = Otri { tri, orient };
            let trisym = m.sym(tri_edge);
            // C: (triangleloop.tri < trisym.tri) ||
            // (trisym.tri == m->dummytri) (triangle.c:7653), the
            // address compare first. The port evaluates the dummy
            // arm first as a deliberate, registered deviation (D18):
            // the packed row-id decode of the reserved sentinel
            // cannot feed the address order, so the dummy arm must
            // short-circuit before it. The boolean is identical (a
            // hull edge is true through the dummy arm either way; an
            // interior edge is visited from the same side through
            // the oracle address order, ADR 0003 section 5, so node
            // creation order is unchanged).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == Ordering::Less
            {
                let torg = m.org(tri_edge);
                let tdest = m.dest(tri_edge);
                // Create a new node in the middle of the edge.
                // Interpolate its attributes (triangle.c:7656-7657).
                let newvertex = m.vx_alloc()?;
                for i in 0..(2 + m.nextras) as usize {
                    m.vx_set_real(
                        newvertex,
                        i,
                        0.5 * (m.vx_real(torg, i) + m.vx_real(tdest, i)),
                    );
                }
                // Set the new node's marker to zero or one, depending on
                // whether it lies on a boundary
                // (triangle.c:7662-7663).
                m.vx_set_mark(newvertex, (trisym.tri == DUMMY_TRI) as i32);
                m.vx_set_type(
                    newvertex,
                    if trisym.tri == DUMMY_TRI {
                        FREEVERTEX
                    } else {
                        SEGMENTVERTEX
                    },
                );
                if b.usesegments {
                    let checkmark = m.tspivot(tri_edge);
                    // If this edge is a segment, transfer the marker to
                    // the new node (triangle.c:7669).
                    if checkmark.ss != DUMMY_SUB {
                        m.vx_set_mark(newvertex, m.sub_mark(checkmark.ss));
                        m.vx_set_type(newvertex, SEGMENTVERTEX);
                    }
                }
                // Record the new node in the (one or two) adjacent
                // elements (triangle.c:7675).
                m.set_tri_midnode(tri, orient as usize, newvertex as u64);
                if trisym.tri != DUMMY_TRI {
                    m.set_tri_midnode(trisym.tri, trisym.orient as usize, newvertex as u64);
                }
            }
        }
        triangleloop = m.triangletraverse(&mut t);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    // Construction-pinned tests (never the C oracle): every fixture is
    // hand-derived geometry, and each asserted outcome follows from the
    // landed C-parity primitives - insertsubseg's reversed sorg/sdest
    // contract, the inlens lens of checkseg4encroach at 20 degrees,
    // deletevertex's origin contract, the pool's LIFO dead stack - not
    // from a reference run.

    #![allow(clippy::float_cmp)] // exact-value pins, the predicates.rs precedent

    use super::*;
    use crate::delaunay::delaunay;
    use crate::slots::DEADVERTEX;
    use crate::transform::insertsubseg;

    /// -q MINANGLE through behavior_update, exactly like quality.rs's
    /// fixture helper; the goodangle the lens and angle gates read.
    fn behavior_q(minangle: f64) -> Behavior {
        let mut b = Behavior {
            quality: true,
            ..Behavior::default_form()
        };
        b.minangle = minangle;
        b.behavior_update();
        b
    }

    /// -q -a MAXAREA (fixedarea): splittriangle's area branch, which is
    /// findcircumcenter in both feature shapes.
    fn behavior_a(maxarea: f64) -> Behavior {
        let mut b = Behavior {
            quality: true,
            fixedarea: true,
            ..Behavior::default_form()
        };
        b.maxarea = maxarea;
        b.behavior_update();
        b
    }

    /// mesh_dim 2 like every stage-1 fixture; no extras.
    fn mesh(b: &Behavior) -> Mesh {
        Mesh::new(b, 16, 2, 0, 0).expect("mesh init")
    }

    fn vxy(m: &mut Mesh, x: f64, y: f64) -> u32 {
        let v = m.vx_alloc().expect("vertex alloc");
        m.set_vx_x(v, x);
        m.set_vx_y(v, y);
        v
    }

    /// A triangle (pa, pb, pc) CCW at orient 0, unbonded (hull edges).
    fn lone_tri(
        m: &mut Mesh,
        usesegments: bool,
        pa: (f64, f64),
        pb: (f64, f64),
        pc: (f64, f64),
    ) -> Otri {
        let va = vxy(m, pa.0, pa.1);
        let vb = vxy(m, pb.0, pb.1);
        let vc = vxy(m, pc.0, pc.1);
        let t = m.maketriangle(usesegments, false).expect("tri alloc");
        m.setorg(t, va);
        m.setdest(t, vb);
        m.setapex(t, vc);
        t
    }

    /// The quality.rs pair fixture: two triangles sharing the edge
    /// (0,0)-(2,0) with a subsegment on it, apexes at the given points.
    /// Returns the subsegment as tspivot hands it over on the first
    /// triangle's orient-0 edge.
    fn seg_pair_fixture(m: &mut Mesh, above: (f64, f64), below: (f64, f64), mark: i32) -> Osub {
        let b = Behavior::default_form();
        let a = vxy(m, 0.0, 0.0);
        let e = vxy(m, 2.0, 0.0);
        let c = vxy(m, above.0, above.1);
        let d = vxy(m, below.0, below.1);
        let t1 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(m, &b, shared, mark).expect("insertsubseg");
        m.tspivot(shared)
    }

    /// The live vertex whose coordinates sit at (x, y) within tol.
    fn find_vertex(m: &Mesh, x: f64, y: f64, tol: f64) -> Option<u32> {
        let mut t = m.vertices.traversal();
        while let Some(v) = m.vertextraverse(&mut t) {
            if (m.vx_x(v) - x).abs() <= tol && (m.vx_y(v) - y).abs() <= tol {
                return Some(v);
            }
        }
        None
    }

    #[test]
    fn tallyencs_enqueues_exactly_the_encroached_segments() {
        // The quality.rs pair fixture: the apex (1, 0.1) is inside the
        // 20-degree lens of the segment (0,0)-(2,0), the apex (1, -10)
        // is far outside, so tallyencs adds exactly one badsubseg row
        // carrying the subsegment handle and its endpoint ids.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let sub = seg_pair_fixture(&mut m, (1.0, 0.1), (1.0, -10.0), 7);
        assert_eq!(m.badsubsegs.live_count(), 0);
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 1);
        assert_eq!(sdecode(m.bad_sub_seg_enc(1)).ss, sub.ss);
        // The row carries (sorg, sdest) of the encroaching side's
        // handle: insertsubseg writes sorg = tridest, sdest = triorg.
        assert_eq!(m.bad_sub_seg_org(1), m.sorg(sub));
        assert_eq!(m.bad_sub_seg_dest(1), m.sdest(sub));

        // A clear pair lens: nothing is enqueued.
        let mut m = mesh(&b);
        seg_pair_fixture(&mut m, (1.0, 10.0), (1.0, -10.0), 7);
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 0);
    }

    #[test]
    fn splitencsegs_splits_an_encroached_segment_at_the_midpoint() {
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        seg_pair_fixture(&mut m, (1.0, 0.1), (1.0, -10.0), 7);
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 1);
        let tri_live = m.triangles.live_count();
        splitencsegs(&mut m, &b, false).expect("splitencsegs");
        // The queue drained: every enqueued row was processed and
        // dealloc'd.
        assert_eq!(m.badsubsegs.live_count(), 0);
        // The split vertex is the binary-exact midpoint (split = 0.5 is
        // exact in f64, so is the coordinate arithmetic).
        let nv = find_vertex(&m, 1.0, 0.0, 1e-12).expect("split vertex");
        assert_eq!(m.vx_x(nv), 1.0);
        assert_eq!(m.vx_y(nv), 0.0);
        assert_eq!(m.vx_mark(nv), 7);
        assert_eq!(m.vx_type(nv), SEGMENTVERTEX);
        // The original subseg row is reused as one half plus a fresh
        // row for the other.
        assert_eq!(m.subsegs.live_count(), 2);
        // Both adjacent triangles split.
        assert_eq!(m.triangles.live_count(), tri_live + 2);
        assert_eq!(m.vertices.live_count(), 5);
        // Unlimited Steiner budget (-1) is untouched by the insert.
        assert_eq!(m.steinerleft, -1);
        // No bad triangles were noted (triflabs is off here).
        assert_eq!(m.badtriangles.live_count(), 0);
    }

    #[test]
    fn splitencsegs_outcome_split_pins_the_raising_site() {
        // Replaces the funnel test's SegSplit arm pin: the raising
        // site (the endpoint check, triangle.c:7337-7342) constructs
        // Error::SegmentRecovery(Split) in place. The fixture
        // hand-enqueues a subsegment whose endpoints have identical
        // coordinates - exactly the row checkseg4encroach writes
        // (quality.rs) - so the split interpolation lands the new
        // vertex on eorg and the endpoint check fires before any
        // insertion.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.5, 0.5);
        let e = vxy(&mut m, 0.5, 0.5); // identical coordinates
        let c = vxy(&mut m, 1.5, 1.5);
        let d = vxy(&mut m, 1.5, -1.5);
        let t1 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
        let sub = m.tspivot(shared);
        let row = m.bad_subseg_alloc().expect("badsubseg row");
        m.set_bad_sub_seg_enc(row, crate::handles::sencode(sub));
        m.set_bad_sub_seg_org(row, m.sorg(sub));
        m.set_bad_sub_seg_dest(row, m.sdest(sub));
        assert_eq!(m.badsubsegs.live_count(), 1);
        let out = splitencsegs(&mut m, &b, false);
        assert!(matches!(
            out,
            Err(Error::SegmentRecovery(SegmentRecoveryKind::Split))
        ));
        // The raising site returns before badsubsegdealloc, so the
        // row stays live exactly as C's does (triangle.c:7342).
        assert_eq!(m.badsubsegs.live_count(), 1);
    }

    #[test]
    fn splitencsegs_chews_a_free_lens_vertex_before_splitting() {
        // A star of three triangles around the free apex c = (1, 0.1)
        // over the encroached segment (0,0)-(2,0): Chew's delete flank
        // removes c first (the degree-3 star collapses to (a, e, d)),
        // deletevertex's quality flank enqueues the merged triangle
        // (its 18.43-degree corner is under the 20-degree bound), and
        // the split vertex reuses c's dealloc'd row (the pool's LIFO).
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 0.1);
        m.vx_set_type(c, FREEVERTEX);
        let d = vxy(&mut m, 3.0, 1.0);
        let c2 = vxy(&mut m, 1.0, -10.0);
        let t1 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, c2);
        let t3 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t3, c);
        m.setdest(t3, e);
        m.setapex(t3, d);
        let t5 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t5, c);
        m.setdest(t5, d);
        m.setapex(t5, a);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 1,
            },
            Otri {
                tri: t3.tri,
                orient: 0,
            },
        );
        m.bond(
            Otri {
                tri: t3.tri,
                orient: 2,
            },
            Otri {
                tri: t5.tri,
                orient: 0,
            },
        );
        m.bond(
            Otri {
                tri: t5.tri,
                orient: 2,
            },
            Otri {
                tri: t1.tri,
                orient: 2,
            },
        );
        insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            7,
        )
        .expect("insertsubseg");
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 1);
        splitencsegs(&mut m, &b, false).expect("splitencsegs");
        // The free lens vertex was deleted and its row reused (LIFO) by
        // the segment split vertex.
        assert_eq!(m.vx_x(c), 1.0);
        assert_eq!(m.vx_y(c), 0.0);
        assert_eq!(m.vx_mark(c), 7);
        assert_eq!(m.vx_type(c), SEGMENTVERTEX);
        // The degree-3 splice deleted two of the star's rows, and both
        // segment-adjacent triangles split.
        assert_eq!(m.triangles.live_count(), 4);
        assert_eq!(m.subsegs.live_count(), 2);
        assert_eq!(m.vertices.live_count(), 5);
        // deletevertex's quality flank enqueued the merged triangle:
        // (0,0)-(2,0)-(3,1) has an 18.43-degree corner.
        assert_eq!(m.badtriangles.live_count(), 1);
        assert_eq!(m.badsubsegs.live_count(), 0);
    }

    #[test]
    fn splitencsegs_concentric_split_at_a_shared_origin_endpoint() {
        // The concentric-circles split: the main segment (0,0)-(5,0) is
        // encroached from below (c2 = (2.5, -0.7) sits inside the
        // 20-degree lens of the whole segment but outside the lens of
        // both halves after the split), and the adjacent subsegment
        // (5,0)-(5,2) shares the endpoint e. The split lands a
        // power-of-two distance (2.0) from e: measured from the stored
        // origin a that is the fraction 1 - 2/5, i.e. (3.0, 0.0).
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 5.0, 0.0);
        let c = vxy(&mut m, 5.0, 2.0);
        let c2 = vxy(&mut m, 2.5, -0.7);
        let t1 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, c2);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            3,
        )
        .expect("insertsubseg");
        // The adjacent subsegment on the (e, c) edge shares endpoint e.
        insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t1.tri,
                orient: 1,
            },
            5,
        )
        .expect("insertsubseg");
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 1);
        splitencsegs(&mut m, &b, false).expect("splitencsegs");
        assert_eq!(m.badsubsegs.live_count(), 0);
        let nv = find_vertex(&m, 3.0, 0.0, 1e-9).expect("concentric split vertex");
        assert_eq!(m.vx_mark(nv), 3);
        assert_eq!(m.vx_type(nv), SEGMENTVERTEX);
        // The main segment's two halves plus the adjacent subsegment.
        assert_eq!(m.subsegs.live_count(), 3);
        assert_eq!(m.triangles.live_count(), 4);
        assert_eq!(m.vertices.live_count(), 5);
    }

    #[test]
    fn splitencsegs_concentric_split_at_a_shared_destination_endpoint() {
        // Mirror of the origin test: the encroachment comes from the
        // stored side (the row keeps the subsegment's own handle), and
        // the adjacent subsegment (2.5, 0.7)-(0, 0) shares the
        // destination endpoint a of the main segment (0,0)-(5,0). The
        // concentric split lands 2.0 away from a: (2.0, 0.0).
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 5.0, 0.0);
        let c1 = vxy(&mut m, 2.5, 0.7);
        let c2 = vxy(&mut m, 2.5, -10.0);
        let t1 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c1);
        let t2 = m.maketriangle(true, false).expect("tri alloc");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, c2);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            3,
        )
        .expect("insertsubseg");
        // The adjacent subsegment on the (c1, a) edge shares endpoint a.
        insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t1.tri,
                orient: 2,
            },
            5,
        )
        .expect("insertsubseg");
        tallyencs(&mut m, &b).expect("tallyencs");
        assert_eq!(m.badsubsegs.live_count(), 1);
        splitencsegs(&mut m, &b, false).expect("splitencsegs");
        assert_eq!(m.badsubsegs.live_count(), 0);
        let nv = find_vertex(&m, 2.0, 0.0, 1e-9).expect("concentric split vertex");
        assert_eq!(m.vx_mark(nv), 3);
        assert_eq!(m.vx_type(nv), SEGMENTVERTEX);
        assert_eq!(m.subsegs.live_count(), 3);
        assert_eq!(m.triangles.live_count(), 4);
        assert_eq!(m.vertices.live_count(), 5);
    }

    #[test]
    fn splittriangle_places_a_circumcenter_under_an_area_constraint() {
        // Runs under both feature settings: the area branch is
        // findcircumcenter in both compile shapes. The lone triangle
        // (0,0)-(2,0)-(1, 1.2) has area 1.2 > 0.5, testtriangle
        // enqueues it, and splittriangle inserts its circumcenter
        // (1, 0.44/2.4): three triangles now, two of which still
        // exceed the bound (area 0.5083 each), which the triflaps
        // flank notes as two more bad-triangle rows.
        let b = behavior_a(0.5);
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (2.0, 0.0), (1.0, 1.2));
        assert_eq!(m.badtriangles.live_count(), 0);
        testtriangle(&mut m, &b, t).expect("testtriangle");
        assert_eq!(m.badtriangles.live_count(), 1);
        splittriangle(&mut m, &b, 1).expect("splittriangle");
        let nv = find_vertex(&m, 1.0, 0.44 / 2.4, 1e-12).expect("circumcenter");
        assert_eq!(m.vx_mark(nv), 0);
        assert_eq!(m.vx_type(nv), FREEVERTEX);
        assert_eq!(m.triangles.live_count(), 3);
        assert_eq!(m.vertices.live_count(), 4);
        // The dequeued row is not dealloc'd by splittriangle (that is
        // enforcequality's else arm); the two oversized children were
        // re-enqueued.
        assert_eq!(m.badtriangles.live_count(), 3);
    }

    #[test]
    #[cfg(feature = "acute")]
    fn splittriangle_routes_through_findnewsplocation_without_an_area_constraint() {
        // Without an area constraint the acute build routes through
        // findNewSPLocation (triangle.c:7437), live since stage-1
        // acute C (#43). The REALLY BAD triangle ((0,0), (2,0),
        // (1,0.1)) has orientation 213, so the Ungor off-center
        // prelude picks the same off-center point the acute.rs
        // family test pins, the row stays allocated through the
        // call, and the insertion splits the lone hull triangle.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (2.0, 0.0), (1.0, 0.1));
        testtriangle(&mut m, &b, t).expect("testtriangle");
        assert_eq!(m.badtriangles.live_count(), 1);
        splittriangle(&mut m, &b, 1).expect("findNewSPLocation");
        let nv = find_vertex(&m, 1.2306141135681588, -2.6438588643184127, 1e-12)
            .expect("offcenter vertex");
        assert_eq!(m.vx_mark(nv), 0);
        assert_eq!(m.vx_type(nv), FREEVERTEX);
        assert_eq!(m.vertices.live_count(), 4);
    }

    #[test]
    #[cfg(not(feature = "acute"))]
    fn splittriangle_reports_a_degenerate_circumcenter_without_the_acute_path() {
        // The NO_ACUTE compile shape: findcircumcenter runs on a
        // collinear triangle, the circumcenter is not finite, and the
        // "left unsplit" check leaves the mesh untouched (the _DEBUG
        // precision printf of that tail is compiled out of the pinned
        // oracle build, so no error surfaces).
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (2.0, 0.0), (1.0, 0.0));
        testtriangle(&mut m, &b, t).expect("testtriangle");
        assert_eq!(m.badtriangles.live_count(), 1);
        splittriangle(&mut m, &b, 1).expect("splittriangle");
        assert_eq!(m.vertices.live_count(), 3);
        assert_eq!(m.triangles.live_count(), 1);
        assert_eq!(m.badtriangles.live_count(), 1);
    }

    #[test]
    fn enforcequality_refines_an_area_constrained_square() {
        // End-to-end under both feature settings: Delaunay over the
        // (0,0)-(1,0)-(1.1,1)-(0,1.3) quadrilateral (2 triangles of
        // area > 0.5), then the -q -a0.05 stage: the bad-triangle
        // queue drains until every triangle is within the area bound,
        // and a fresh tallyfaces over the restarted pool finds nothing.
        let b = behavior_a(0.05);
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.1, 1.0), (0.0, 1.3)] {
            vxy(&mut m, x, y);
        }
        delaunay(&mut m, &b).expect("delaunay");
        assert_eq!(m.triangles.live_count(), 2);
        enforcequality(&mut m, &b).expect("enforcequality");
        assert!(m.triangles.live_count() > 2);
        assert_eq!(m.badtriangles.live_count(), 0);
        assert_eq!(m.badsubsegs.live_count(), 0);
        // Re-tally after the pool restart: every triangle is within
        // the bound now, nothing is enqueued.
        m.badtriangles.restart();
        tallyfaces(&mut m, &b).expect("tallyfaces");
        assert_eq!(m.badtriangles.live_count(), 0);
    }

    #[test]
    #[cfg(feature = "acute")]
    fn enforcequality_refines_skinny_triangles_through_the_acute_path() {
        // The acute build's twin of the NO_ACUTE test below: the
        // Ruppert loop runs to completion under a 200-point Steiner
        // budget, every split goes through findNewSPLocation
        // (triangle.c:7437), the queue drains, the triangle count
        // grows, and the funneled status stays Ok.
        let mut b = behavior_q(20.0);
        b.steiner = 200;
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (2.0, 0.0), (0.9, 0.05), (1.0, 5.0)] {
            vxy(&mut m, x, y);
        }
        delaunay(&mut m, &b).expect("delaunay");
        let before = m.triangles.live_count();
        enforcequality(&mut m, &b).expect("enforcequality");
        assert!(m.triangles.live_count() > before);
        assert_eq!(m.badtriangles.live_count(), 0);
        assert_eq!(m.badsubsegs.live_count(), 0);
    }

    #[test]
    #[cfg(not(feature = "acute"))]
    fn enforcequality_repairs_skinny_triangles_without_the_acute_path() {
        // The NO_ACUTE build: the Ruppert loop runs to completion
        // under a 200-point Steiner budget (20 degrees is under the
        // termination bound); the queue drains, the triangle count
        // grows, and the funneled status stays Ok.
        let mut b = behavior_q(20.0);
        b.steiner = 200;
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (2.0, 0.0), (0.9, 0.05), (1.0, 5.0)] {
            vxy(&mut m, x, y);
        }
        delaunay(&mut m, &b).expect("delaunay");
        let before = m.triangles.live_count();
        enforcequality(&mut m, &b).expect("enforcequality");
        assert!(m.triangles.live_count() > before);
        assert_eq!(m.badtriangles.live_count(), 0);
        assert_eq!(m.badsubsegs.live_count(), 0);
    }

    #[test]
    fn highorder_creates_one_node_per_edge_of_a_unit_square() {
        // Two triangles over the square (0,0)-(1,0)-(1,1)-(0,1)
        // sharing the diagonal a-c: five edges, each visited exactly
        // once (the interior edge only from the lower-id triangle,
        // C's raw pointer compare through the oracle address order).
        // The five new nodes take fresh ids in traversal order, the
        // four hull nodes carry mark 1 / FREEVERTEX, the interior
        // node mark 0 / SEGMENTVERTEX, and both adjacent rows record
        // it.
        let mut b = Behavior::default_form();
        b.order = 2;
        b.behavior_update();
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 0.0, 1.0);
        let t1 = m.maketriangle(false, false).expect("tri alloc");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(false, false).expect("tri alloc");
        m.setorg(t2, a);
        m.setdest(t2, c);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 2,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        let hi = m.tri_shape.highorderindex;
        assert_eq!(hi, 6);
        highorder(&mut m, &b).expect("highorder");
        assert_eq!(m.vertices.live_count(), 9);
        // Fresh ids in traversal order: t1's three edges get 5, 6, 7
        // (the interior edge is its orient-2 edge), then t2's two hull
        // edges get 8 and 9.
        let interior = 7u32;
        assert_eq!(m.vx_x(interior), 0.5);
        assert_eq!(m.vx_y(interior), 0.5);
        assert_eq!(m.vx_mark(interior), 0);
        assert_eq!(m.vx_type(interior), SEGMENTVERTEX);
        // Both adjacent element rows record the interior node on the
        // shared edge's orientation slots.
        assert_eq!(m.tri_midnode(t1.tri, 2), u64::from(interior));
        assert_eq!(m.tri_midnode(t2.tri, 0), u64::from(interior));
        // The hull nodes: mark 1, FREEVERTEX, recorded on their edges.
        for (tri, orient, x, y) in [
            (t1.tri, 0usize, 0.5, 0.0),
            (t1.tri, 1, 1.0, 0.5),
            (t2.tri, 1, 0.5, 1.0),
            (t2.tri, 2, 0.0, 0.5),
        ] {
            let node = m.tri_midnode(tri, orient) as u32;
            let v = find_vertex(&m, x, y, 1e-12).expect("hull node");
            assert_eq!(node, v);
            assert_eq!(m.vx_mark(v), 1);
            assert_eq!(m.vx_type(v), FREEVERTEX);
        }
    }

    #[test]
    fn highorder_cannot_allocate_the_dead_vertex_rows() {
        // triangle.c:7639: the dead-item stack is cleared so the extra
        // nodes are all fresh rows; the dealloc'd vertex keeps its row
        // (still dead), and the three edge nodes take ids beyond the
        // allocation frontier.
        let mut b = Behavior::default_form();
        b.order = 2;
        b.behavior_update();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        let dead = vxy(&mut m, 9.0, 9.0);
        m.vx_dealloc(dead);
        // The allocation frontier sits one past the dead row (it was
        // this fixture's last allocation; next_fresh is monotonic).
        let fresh = dead + 1;
        highorder(&mut m, &b).expect("highorder");
        assert_eq!(m.vertices.live_count(), 6);
        // The dead row was not handed out again.
        assert_eq!(m.vx_type(dead), DEADVERTEX);
        for (orient, want) in [(0usize, fresh), (1, fresh + 1), (2, fresh + 2)] {
            assert_eq!(m.tri_midnode(t.tri, orient), u64::from(want));
        }
    }

    #[test]
    fn highorder_transfers_segment_markers_to_edge_nodes() {
        // -p2: the lone triangle's (0,0)-(2,0) edge carries subsegment
        // 7; the edge node inherits mark 7 / SEGMENTVERTEX, the two
        // free hull nodes keep mark 1 / FREEVERTEX.
        let mut b = Behavior::default_form();
        b.poly = true;
        b.usesegments = true;
        b.order = 2;
        b.behavior_update();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, true, (0.0, 0.0), (2.0, 0.0), (1.0, 1.0));
        insertsubseg(&mut m, &b, t, 7).expect("insertsubseg");
        let hi = m.tri_shape.highorderindex;
        assert_eq!(hi, 9);
        highorder(&mut m, &b).expect("highorder");
        assert_eq!(m.vertices.live_count(), 6);
        // The segment edge's node.
        let segnode = m.tri_midnode(t.tri, 0) as u32;
        assert_eq!(m.vx_x(segnode), 1.0);
        assert_eq!(m.vx_y(segnode), 0.0);
        assert_eq!(m.vx_mark(segnode), 7);
        assert_eq!(m.vx_type(segnode), SEGMENTVERTEX);
        // The other two edges: boundary nodes.
        for (orient, x, y) in [(1usize, 1.5, 0.5), (2, 0.5, 0.5)] {
            let node = m.tri_midnode(t.tri, orient) as u32;
            assert_eq!(m.vx_mark(node), 1);
            assert_eq!(m.vx_type(node), FREEVERTEX);
            assert_eq!(m.vx_x(node), x);
            assert_eq!(m.vx_y(node), y);
        }
    }
}
