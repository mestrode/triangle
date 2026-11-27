//! The eps_writer.c unit (src/Triangle/eps_writer.c with its private
//! header src/Triangle/private/eps_writer.h): the PostScript
//! renderer file_write_eps composes (triangle_io.c:331-347). Every
//! fprintf of the C unit is mirrored byte for byte through the
//! formatter seam; the unchecked fprintf return values surface as
//! io::Result like the other writers (the register's writer-seam
//! row).
//!
//! C anchors: eps_write_header:14, eps_write_trailer:32,
//! eps_draw_clip:43, eps_draw_edges:87, eps_draw_segments:132,
//! eps_draw_points:161, eps_set_color:192, eps_set_stroke:198,
//! eps_update_metrics:205; the rint macro (eps_writer.h:6) and the
//! TX/TY transforms (eps_writer.h:9-10).
//!
//! Two C facts the port mirrors exactly:
//! - C's rint(x) macro is the MSC replacement `(int)((x) + 0.5)`,
//!   a truncation toward zero, not libc's round-to-nearest. The port
//!   writes the same truncation (`as i32` on the f64 sum).
//! - The edge-emit gate (eps_draw_edges, eps_writer.c:117) evaluates
//!   C's raw address compare first, like triangle.c:7653 and
//!   triangle_io.c:232. The port swaps the conjuncts (the D18/D28
//!   precedent, register row D30): the dummy arm must short-circuit
//!   before the oracle address order, which cannot evaluate the
//!   reserved DUMMY_TRI sentinel row. Equivalent on every input
//!   (ADR 0003 section 5).
//!
//! The oracle corpus does not diff .eps output (tools/oracle/
//! generate_corpus.py emits no .eps expectations), so the harness
//! gates nothing here: construction tests over the printf spec gate
//! this module instead.

// Clippy: the unit keeps C's fprintf format strings verbatim,
// trailing \n included, and eps_update_metrics keeps C's
// expanded adds (eps_writer.c:205-233) - writeln!/+= would
// rewrite the C literals.
#![allow(clippy::write_with_newline)]
#![allow(clippy::assign_op_pattern)]

use std::io::Write;

use crate::formatter::g as fmt_g;
use crate::mesh::handles::{Osub, Otri, DUMMY_TRI};
use crate::mesh::slots::UNDEADVERTEX;
use crate::mesh::Mesh;

/// C's rect (triangle.h: the metric rectangle eps_update_metrics
/// shrinks and the TX/TY transforms read).
pub(crate) struct Rect {
    pub(crate) xmin: f64,
    pub(crate) ymin: f64,
    pub(crate) xmax: f64,
    pub(crate) ymax: f64,
}

/// The rint macro (eps_writer.h:6): MSC's `(int)((x) + 0.5)`, a
/// truncation toward zero (NOT libc rint's round-half-to-even).
fn c_rint(x: f64) -> i32 {
    (x + 0.5) as i32
}

/// TX (eps_writer.h:9): map a mesh x to a page x, truncated like C's
/// (int)floor(...).
fn tx(x: f64, ps: &Rect, ms: &Rect) -> i32 {
    (((ms.xmax - x) * ps.xmin + (x - ms.xmin) * ps.xmax) / (ms.xmax - ms.xmin)).floor() as i32
}

/// TY (eps_writer.h:10).
fn ty(y: f64, ps: &Rect, ms: &Rect) -> i32 {
    (((ms.ymax - y) * ps.ymin + (y - ms.ymin) * ps.ymax) / (ms.ymax - ms.ymin)).floor() as i32
}

/// eps_write_header (eps_writer.c:14-30).
pub(crate) fn eps_write_header(
    file: &mut dyn Write,
    title: &str,
    ps: &Rect,
) -> std::io::Result<()> {
    write!(file, "%!PS-Adobe-3.0 EPSF-3.0\n")?;
    write!(file, "%%Creator: libtriangle\n")?;
    write!(file, "%%Title: {}\n", title)?;
    write!(file, "%%Pages: 1\n")?;
    write!(
        file,
        "%%BoundingBox:  {}  {}  {}  {}\n",
        ps.xmin as i32, ps.ymin as i32, ps.xmax as i32, ps.ymax as i32
    )?;
    write!(file, "%%Document-Fonts: Times-Roman\n")?;
    write!(file, "%%LanguageLevel: 1\n")?;
    write!(file, "%%EndComments\n")?;
    write!(file, "%%BeginProlog\n")?;
    write!(file, "/inch {{72 mul}} def\n")?;
    write!(file, "%%EndProlog\n")?;
    write!(file, "%%Page: 1 1\n")?;
    Ok(())
}

/// eps_write_trailer (eps_writer.c:32-40).
pub(crate) fn eps_write_trailer(file: &mut dyn Write) -> std::io::Result<()> {
    write!(file, "%\n")?;
    write!(file, "restore  showpage\n")?;
    write!(file, "%\n")?;
    write!(file, "%  End of page.\n")?;
    write!(file, "%\n")?;
    write!(file, "%%Trailer\n")?;
    write!(file, "%%EOF\n")?;
    Ok(())
}

/// eps_draw_clip (eps_writer.c:43-85).
pub(crate) fn eps_draw_clip(file: &mut dyn Write, ps: &Rect, clip: &Rect) -> std::io::Result<()> {
    write!(file, "save\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Set the RGB color to very light gray.\n")?;
    write!(file, "%\n")?;
    write!(file, "0.900  0.900  0.900 setrgbcolor\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Draw a gray border around the page.\n")?;
    write!(file, "%\n")?;
    write!(file, "newpath\n")?;
    write!(file, "  {}  {}  moveto\n", ps.xmin as i32, ps.ymin as i32)?;
    write!(file, "  {}  {}  lineto\n", ps.xmax as i32, ps.ymin as i32)?;
    write!(file, "  {}  {}  lineto\n", ps.xmax as i32, ps.ymax as i32)?;
    write!(file, "  {}  {}  lineto\n", ps.xmin as i32, ps.ymax as i32)?;
    write!(file, "  {}  {}  lineto\n", ps.xmin as i32, ps.ymin as i32)?;
    write!(file, "stroke\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Set the RGB color to black.\n")?;
    write!(file, "%\n")?;
    write!(file, "0.000  0.000  0.000 setrgbcolor\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Set the font and its size.\n")?;
    write!(file, "%\n")?;
    write!(file, "/Times-Roman findfont\n")?;
    write!(file, "0.50 inch scalefont\n")?;
    write!(file, "setfont\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Print a title.\n")?;
    write!(file, "%\n")?;
    write!(file, "%210  702  moveto\n")?;
    write!(file, "%(Triangulation)  show\n")?;
    write!(file, "%\n")?;
    write!(file, "%  Define a clipping polygon.\n")?;
    write!(file, "%\n")?;
    write!(file, "newpath\n")?;
    write!(
        file,
        "  {}  {}  moveto\n",
        clip.xmin as i32, clip.ymin as i32
    )?;
    write!(
        file,
        "  {}  {}  lineto\n",
        clip.xmax as i32, clip.ymin as i32
    )?;
    write!(
        file,
        "  {}  {}  lineto\n",
        clip.xmax as i32, clip.ymax as i32
    )?;
    write!(
        file,
        "  {}  {}  lineto\n",
        clip.xmin as i32, clip.ymax as i32
    )?;
    write!(
        file,
        "  {}  {}  lineto\n",
        clip.xmin as i32, clip.ymin as i32
    )?;
    write!(file, "clip newpath\n")?;
    Ok(())
}

/// eps_draw_edges (eps_writer.c:87-130).
pub(crate) fn eps_draw_edges(
    file: &mut dyn Write,
    m: &Mesh,
    ps: &Rect,
    ms: &Rect,
) -> std::io::Result<()> {
    write!(file, "%\n")?;
    write!(file, "%  Draw the triangles (mesh edges).\n")?;
    write!(file, "%\n")?;

    eps_set_stroke(file, 0.6, 0.6, 0.6, 0.4)?;

    write!(
        file,
        "/L {{\n2 dict begin\n/y2 exch def\n/x2 exch def\n/y1 exch def\n/x1 exch def\n\
         gsave\nnewpath x1 y1 moveto x2 y2 lineto stroke\ngrestore\nend\n}} def\n"
    )?;

    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        for orient in 0..3u8 {
            let e = Otri { tri, orient };
            let trisym = m.sym(e);
            // C: (e.tri < trisym.tri) || (trisym.tri == m->dummytri)
            // (eps_writer.c:117), the raw compare first. D30: the
            // dummy arm evaluates first (the D18/D28 precedent).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == std::cmp::Ordering::Less
            {
                let p = m.org(e);
                let q = m.dest(e);
                write!(
                    file,
                    "{} {} {} {} L\n",
                    tx(m.vx_x(p), ps, ms),
                    ty(m.vx_y(p), ps, ms),
                    tx(m.vx_x(q), ps, ms),
                    ty(m.vx_y(q), ps, ms)
                )?;
            }
        }
    }
    Ok(())
}

/// eps_draw_segments (eps_writer.c:132-159).
pub(crate) fn eps_draw_segments(
    file: &mut dyn Write,
    m: &Mesh,
    ps: &Rect,
    ms: &Rect,
) -> std::io::Result<()> {
    write!(file, "%\n")?;
    write!(file, "%  Draw the segments.\n")?;
    write!(file, "%\n")?;

    eps_set_stroke(file, 0.27, 0.5, 0.7, 0.8)?;

    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.subsegtraverse(&mut t) {
        let e = Osub { ss, ssorient: 0 };
        let p = m.sorg(e);
        let q = m.sdest(e);
        write!(
            file,
            "{} {} {} {} L\n",
            tx(m.vx_x(p), ps, ms),
            ty(m.vx_y(p), ps, ms),
            tx(m.vx_x(q), ps, ms),
            ty(m.vx_y(q), ps, ms)
        )?;
    }
    Ok(())
}

/// eps_draw_points (eps_writer.c:161-190).
pub(crate) fn eps_draw_points(
    file: &mut dyn Write,
    m: &Mesh,
    ps: &Rect,
    ms: &Rect,
) -> std::io::Result<()> {
    write!(file, "%\n")?;
    write!(file, "%  Draw the vertices.\n")?;
    write!(file, "%\n")?;

    eps_set_color(file, 0.0, 0.4, 0.0)?;

    write!(
        file,
        "/P {{\n2 dict begin\n/y exch def\n/x exch def\n\
         gsave\nnewpath x y 1 0 360 arc fill\ngrestore\nend\n}} def\n"
    )?;

    let mut t = m.vertices.traversal();
    while let Some(v) = m.vertextraverse(&mut t) {
        if m.vx_type(v) != UNDEADVERTEX {
            write!(
                file,
                "{} {} P\n",
                tx(m.vx_x(v), ps, ms),
                ty(m.vx_y(v), ps, ms)
            )?;
        }
    }
    Ok(())
}

/// eps_set_color (eps_writer.c:192-196). C passes the float arguments
/// through varargs, so they promote to double before %.3g; the port
/// converts at the format call.
pub(crate) fn eps_set_color(file: &mut dyn Write, r: f32, g: f32, b: f32) -> std::io::Result<()> {
    write!(
        file,
        "{} {} {} setrgbcolor\n",
        fmt_g(f64::from(r), 3),
        fmt_g(f64::from(g), 3),
        fmt_g(f64::from(b), 3)
    )?;
    write!(file, "%\n")?;
    Ok(())
}

/// eps_set_stroke (eps_writer.c:198-203).
pub(crate) fn eps_set_stroke(
    file: &mut dyn Write,
    r: f32,
    g: f32,
    b: f32,
    width: f32,
) -> std::io::Result<()> {
    write!(
        file,
        "{} {} {} setrgbcolor\n",
        fmt_g(f64::from(r), 3),
        fmt_g(f64::from(g), 3),
        fmt_g(f64::from(b), 3)
    )?;
    write!(file, "{} setlinewidth\n", fmt_g(f64::from(width), 3))?;
    write!(file, "%\n")?;
    Ok(())
}

/// eps_update_metrics (eps_writer.c:205-241): the 5% margins on each
/// side, then the aspect fit that shrinks the page and clip rects by
/// the rint-macro delta.
#[allow(unused_assignments)] // C stores the fitted scale back (eps_writer.c:234, 241); the value is dead at the return but the store is transliterated.
pub(crate) fn eps_update_metrics(ps: &mut Rect, clip: &mut Rect, ms: &mut Rect) {
    // Enlarge width 5% on each side.
    let mut x_scale = ms.xmax - ms.xmin;
    ms.xmax = ms.xmax + 0.05 * x_scale;
    ms.xmin = ms.xmin - 0.05 * x_scale;
    x_scale = ms.xmax - ms.xmin;

    // Enlarge height 5% on each side.
    let mut y_scale = ms.ymax - ms.ymin;
    ms.ymax = ms.ymax + 0.05 * y_scale;
    ms.ymin = ms.ymin - 0.05 * y_scale;
    y_scale = ms.ymax - ms.ymin;

    if x_scale < y_scale {
        let delta = c_rint((ps.xmax - ps.xmin) * (y_scale - x_scale) / (2.0 * y_scale));

        ps.xmax = ps.xmax - f64::from(delta);
        ps.xmin = ps.xmin + f64::from(delta);

        clip.xmax = clip.xmax - f64::from(delta);
        clip.xmin = clip.xmin + f64::from(delta);

        x_scale = y_scale;
    } else {
        let delta = c_rint((ps.ymax - ps.ymin) * (x_scale - y_scale) / (2.0 * x_scale));

        ps.ymax = ps.ymax - f64::from(delta);
        ps.ymin = ps.ymin + f64::from(delta);

        clip.ymax = clip.ymax - f64::from(delta);
        clip.ymin = clip.ymin + f64::from(delta);

        y_scale = x_scale;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::Options;

    fn behavior_with(setup: impl FnOnce(&mut Options)) -> crate::options::Behavior {
        let mut opts = Options::new();
        setup(&mut opts);
        opts.validate().expect("validate")
    }

    /// The two-triangle unit square of the io/mod.rs tests (same shape as
    /// the io/arrays.rs fixture): vertices (0,0), (1,0), (1,1), (0,1)
    /// bonded across t1's orient-2 edge.
    fn square_two_triangles(b: &crate::options::Behavior) -> (Mesh, u32, u32, [u32; 4]) {
        let mut m = Mesh::new(b, 8, 2, 0, 0).expect("mesh init");
        let a = m.vx_alloc().expect("vx");
        m.set_vx_x(a, 0.0);
        m.set_vx_y(a, 0.0);
        let e = m.vx_alloc().expect("vx");
        m.set_vx_x(e, 1.0);
        m.set_vx_y(e, 0.0);
        let c = m.vx_alloc().expect("vx");
        m.set_vx_x(c, 1.0);
        m.set_vx_y(c, 1.0);
        let d = m.vx_alloc().expect("vx");
        m.set_vx_x(d, 0.0);
        m.set_vx_y(d, 1.0);
        let t1 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
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
        (m, t1.tri, t2.tri, [a, e, c, d])
    }

    fn page_and_clip() -> (Rect, Rect) {
        (
            Rect {
                xmin: 36.0,
                ymin: 126.0,
                xmax: 576.0,
                ymax: 666.0,
            },
            Rect {
                xmin: 18.0,
                ymin: 108.0,
                xmax: 594.0,
                ymax: 684.0,
            },
        )
    }

    #[test]
    fn update_metrics_square_mesh_leaves_the_page_alone() {
        let (mut ps, mut clip) = page_and_clip();
        let mut ms = Rect {
            xmin: 0.0,
            ymin: 0.0,
            xmax: 1.0,
            ymax: 1.0,
        };
        eps_update_metrics(&mut ps, &mut clip, &mut ms);
        // The 5% margins land on the mesh rect; equal aspect scales
        // shrink nothing (the rint delta is rint(0.0) = (int)0.5 = 0,
        // the macro's truncation).
        assert_eq!(ms.xmin, -0.05);
        assert_eq!(ms.ymin, -0.05);
        assert_eq!(ms.xmax, 1.05);
        assert_eq!(ps.xmin, 36.0);
        assert_eq!(ps.xmax, 576.0);
        assert_eq!(ps.ymin, 126.0);
        assert_eq!(ps.ymax, 666.0);
        assert_eq!(clip.xmin, 18.0);
        assert_eq!(clip.ymax, 684.0);
    }

    #[test]
    fn update_metrics_wide_mesh_shrinks_the_page_height() {
        let (mut ps, mut clip) = page_and_clip();
        let mut ms = Rect {
            xmin: 0.0,
            ymin: 0.0,
            xmax: 2.0,
            ymax: 1.0,
        };
        eps_update_metrics(&mut ps, &mut clip, &mut ms);
        // x_scale 2.2 vs y_scale 1.1: the else branch, delta =
        // rint(540 * 1.1 / 4.4) = rint(135.0) = (int)135.5 = 135 (the
        // macro truncates the half toward zero).
        assert_eq!(ps.ymin, 126.0 + 135.0);
        assert_eq!(ps.ymax, 666.0 - 135.0);
        assert_eq!(clip.ymin, 108.0 + 135.0);
        assert_eq!(clip.ymax, 684.0 - 135.0);
        assert_eq!(ps.xmin, 36.0);
        assert_eq!(ps.xmax, 576.0);
    }

    #[test]
    fn tx_ty_map_the_unit_square_to_the_page() {
        // After the 5% margins of a 1x1 mesh: ms = (-0.05, -0.05,
        // 1.05, 1.05); page unchanged (equal aspect). TX(0) =
        // floor(66.6 / 1.1) = 60, TY(0) = floor(165.6 / 1.1) = 150,
        // TX(1) = floor(606.6 / 1.1) = 551, TY(1) = floor(705.6 / 1.1)
        // = 641.
        let (ps, _clip) = page_and_clip();
        let ms = Rect {
            xmin: -0.05,
            ymin: -0.05,
            xmax: 1.05,
            ymax: 1.05,
        };
        assert_eq!(tx(0.0, &ps, &ms), 60);
        assert_eq!(ty(0.0, &ps, &ms), 150);
        assert_eq!(tx(1.0, &ps, &ms), 551);
        assert_eq!(ty(1.0, &ps, &ms), 641);
    }

    #[test]
    fn eps_set_color_and_stroke_render_the_floats() {
        // %.3g over the float-to-double promotions: C's f32 literals
        // are not exact decimals (0.6f is 0.60000002384...), but
        // three significant digits round them back to the short
        // forms.
        let mut out: Vec<u8> = Vec::new();
        eps_set_color(&mut out, 0.0, 0.4, 0.0).expect("color");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "0 0.4 0 setrgbcolor\n%\n"
        );
        let mut out: Vec<u8> = Vec::new();
        eps_set_stroke(&mut out, 0.27, 0.5, 0.7, 0.8).expect("stroke");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "0.27 0.5 0.7 setrgbcolor\n0.8 setlinewidth\n%\n"
        );
    }

    #[test]
    fn file_write_eps_renders_the_square_byte_for_byte() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b);
        // Mesh bounds (file_write_eps reads them; the fixture does
        // not triangulate, so they are set here like transfernodes
        // would).
        m.xmin = 0.0;
        m.ymin = 0.0;
        m.xmax = 1.0;
        m.ymax = 1.0;
        let mut out: Vec<u8> = Vec::new();
        crate::io::file_write_eps(&mut m, &b, &mut out).expect("eps");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "%!PS-Adobe-3.0 EPSF-3.0\n\
             %%Creator: libtriangle\n\
             %%Title: mesh.eps\n\
             %%Pages: 1\n\
             %%BoundingBox:  36  126  576  666\n\
             %%Document-Fonts: Times-Roman\n\
             %%LanguageLevel: 1\n\
             %%EndComments\n\
             %%BeginProlog\n\
             /inch {72 mul} def\n\
             %%EndProlog\n\
             %%Page: 1 1\n\
             save\n\
             %\n\
             %  Set the RGB color to very light gray.\n\
             %\n\
             0.900  0.900  0.900 setrgbcolor\n\
             %\n\
             %  Draw a gray border around the page.\n\
             %\n\
             newpath\n\
             \x20\x2036  126  moveto\n\
             \x20\x20576  126  lineto\n\
             \x20\x20576  666  lineto\n\
             \x20\x2036  666  lineto\n\
             \x20\x2036  126  lineto\n\
             stroke\n\
             %\n\
             %  Set the RGB color to black.\n\
             %\n\
             0.000  0.000  0.000 setrgbcolor\n\
             %\n\
             %  Set the font and its size.\n\
             %\n\
             /Times-Roman findfont\n\
             0.50 inch scalefont\n\
             setfont\n\
             %\n\
             %  Print a title.\n\
             %\n\
             %210  702  moveto\n\
             %(Triangulation)  show\n\
             %\n\
             %  Define a clipping polygon.\n\
             %\n\
             newpath\n\
             \x20\x2018  108  moveto\n\
             \x20\x20594  108  lineto\n\
             \x20\x20594  684  lineto\n\
             \x20\x2018  684  lineto\n\
             \x20\x2018  108  lineto\n\
             clip newpath\n\
             %\n\
             %  Draw the triangles (mesh edges).\n\
             %\n\
             0.6 0.6 0.6 setrgbcolor\n\
             0.4 setlinewidth\n\
             %\n\
             /L {\n\
             2 dict begin\n\
             /y2 exch def\n\
             /x2 exch def\n\
             /y1 exch def\n\
             /x1 exch def\n\
             gsave\n\
             newpath x1 y1 moveto x2 y2 lineto stroke\n\
             grestore\n\
             end\n\
             } def\n\
             60 150 551 150 L\n\
             551 150 551 641 L\n\
             551 641 60 150 L\n\
             551 641 60 641 L\n\
             60 641 60 150 L\n\
             %\n\
             %  Draw the segments.\n\
             %\n\
             0.27 0.5 0.7 setrgbcolor\n\
             0.8 setlinewidth\n\
             %\n\
             %\n\
             %  Draw the vertices.\n\
             %\n\
             0 0.4 0 setrgbcolor\n\
             %\n\
             /P {\n\
             2 dict begin\n\
             /y exch def\n\
             /x exch def\n\
             gsave\n\
             newpath x y 1 0 360 arc fill\n\
             grestore\n\
             end\n\
             } def\n\
             60 150 P\n\
             551 150 P\n\
             551 641 P\n\
             60 641 P\n\
             %\n\
             restore  showpage\n\
             %\n\
             %  End of page.\n\
             %\n\
             %%Trailer\n\
             %%EOF\n"
        );
    }
}
