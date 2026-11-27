//! The public options surface (ADR 0004, sections 5.1 and 5.4):
//! 'Options', the public replacement of C's 'struct behavior'
//! (triangle.h:371-470); 'Options::new', the exact 'parsecommandline("")
//! defaults; 'Options::parse', the transliterated character loop of
//! parsecommandline (triangle.c:389-526); 'Options::validate', the
//! transliterated 'triangle_check_behavior' (triangle_helper.c:17-23)
//! plus the derived-field tail of 'behavior_update' (triangle.c:275-303),
//! normalizing into the internal 'Behavior' that the transliterated
//! core consumes; the 'Unsuitable' callback trait and its
//! 'VertexSample' values (the triunsuitable_user_func mirror, D9).
//!
//! The builder methods mirror the switch families one-to-one: every C
//! switch letter is documented on its method, nothing C parses is
//! dropped, and no new options are invented (ADR 0004 section 5.1).
//! Derived fields (usesegments, goodangle, maxgoodangle, offconstant and
//! the vararea/regionattrib/weighted corrections) are not public setter
//! surface; validate() computes them so every entry point consumes a
//! normalized Behavior exactly where C consumes a behavior struct after
//! its parsecommandline/behavior_update tail.

// The Behavior fields mirror struct behavior one-to-one ahead of the
// modules that read them; the blanket silences that ahead-of-use
// dead_code and is removed field by field as the core lands.
#![allow(dead_code)]

use std::f64::consts::PI;

use crate::error::Error;

/// The triunsuitable_user_func callback values (ADR 0004 section 5.4,
/// deviation D9): copied vertex data only, never references into the
/// mesh rows, so no callback can corrupt storage behind the chokepoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexSample {
    pub x: f64,
    pub y: f64,
    pub marker: i32,
}

/// The user-defined triangle sizing callback (C:
/// 'int (*triunsuitable_user_func)(vertex, vertex, vertex, REAL)',
/// invoked inside testtriangle). Object-safe, one method, '&mut self'
/// (stateful sizing functions are the norm), stored in Options as an
/// Option<Box<dyn Unsuitable>>. No Send/Sync supertraits today: the
/// parallelism ruling (#12, read-only for this shape) lands inside
/// testtriangle and every candidate ruling keeps this shape.
pub trait Unsuitable {
    /// Returns true when the triangle must be refined further
    /// (C: nonzero).
    fn is_unsuitable(
        &mut self,
        org: VertexSample,
        dest: VertexSample,
        apex: VertexSample,
        area: f64,
    ) -> bool;
}

/// The public options surface, one struct mirroring the field groups of
/// C's 'struct behavior' (triangle.h:371-470). Fields are private;
/// construction goes through 'Options::new' (defaults), the builder
/// setters (one per switch family, the switch letter documented on
/// each) or 'Options::parse'. The normalized internal form the core
/// consumes is 'Behavior', produced by 'validate()'.
///
/// # Examples
///
/// The options string of the C candidate contract parses into the
/// switch families:
///
/// ```
/// use triangle::Options;
///
/// assert!(Options::parse("pq30a0.5").is_ok());
/// ```
pub struct Options {
    /// -p: triangulate a PSLG. C b->poly.
    poly: bool,
    /// -r: refine a previously generated mesh. C b->refine.
    refine: bool,
    /// -q (also set by -U, -a, -u, -D, -L). C b->quality.
    quality: bool,
    /// -a without a number: area constraint per triangle. C b->vararea.
    vararea: bool,
    /// -a with a number. C b->fixedarea.
    fixedarea: bool,
    /// -u: user-defined test, via the Unsuitable callback. C b->usertest.
    usertest: bool,
    /// -A: regional attributes. C b->regionattrib.
    regionattrib: bool,
    /// -c: enclose convex hull with segments. C b->convex.
    convex: bool,
    /// -w (1) / -W (2): weighted / regular triangulation. C b->weighted.
    weighted: i32,
    /// -j: jettison unused vertices. C b->jettison.
    jettison: bool,
    /// -z: number items from zero (default one). C b->firstnumber.
    firstnumber: i32,
    /// -n: generate element neighbors. C b->neighbors.
    neighbors: bool,
    /// -B: suppress boundary output. C b->nobound.
    nobound: bool,
    /// -O: ignore holes. C b->noholes.
    noholes: bool,
    /// -X: suppress exact arithmetic. C b->noexact.
    noexact: bool,
    /// -D / -L: conforming Delaunay (also sets quality). C b->conformdel.
    conformdel: bool,
    /// -i: incremental method. C b->incremental.
    incremental: bool,
    /// -F: sweepline method. C b->sweepline.
    sweepline: bool,
    /// -l off (default on): alternating cuts / Dwyer. C b->dwyer.
    dwyer: bool,
    /// -s: force segments by splitting. C b->splitseg.
    splitseg: bool,
    /// -o2: element order. C b->order.
    order: i32,
    /// -Y (repeatable, incrementing). C b->nobisect.
    nobisect: i32,
    /// -S N (0 = disabled; -1 = unlimited, the default). C b->steiner.
    steiner: i32,
    /// -qN: minimum angle bound in degrees. C b->minangle.
    minangle: f64,
    /// -a N: maximum area bound. C b->maxarea.
    maxarea: f64,
    /// -UN: maximum angle bound (acute feature only, ADR 0004 section 2;
    /// the mirror of C's #ifndef NO_ACUTE field). C b->maxangle.
    #[cfg(feature = "acute")]
    maxangle: f64,
    /// The -u sizing callback. C b->triunsuitable_user_func.
    user_func: Option<Box<dyn Unsuitable>>,
}

impl Options {
    /// The exact 'parsecommandline("")' defaults of C
    /// (triangle.c:369-387): dwyer on, firstnumber 1, steiner
    /// unlimited (-1), order 1, minangle 0.0, maxangle 0.0, maxarea
    /// -1.0 decimal.
    pub fn new() -> Options {
        Options {
            poly: false,
            refine: false,
            quality: false,
            vararea: false,
            fixedarea: false,
            usertest: false,
            regionattrib: false,
            convex: false,
            weighted: 0,
            jettison: false,
            firstnumber: 1,
            neighbors: false,
            nobound: false,
            noholes: false,
            noexact: false,
            conformdel: false,
            incremental: false,
            sweepline: false,
            dwyer: true,
            splitseg: false,
            order: 1,
            nobisect: 0,
            steiner: -1,
            minangle: 0.0,
            maxarea: -1.0,
            #[cfg(feature = "acute")]
            maxangle: 0.0,
            user_func: None,
        }
    }

    /// The transliterated character loop of C's parsecommandline
    /// (triangle.c:389-526), defaults included: every 'if' of the C
    /// loop is an independent branch here too, the inline number
    /// parsing mirrors C's digit/dot runs through strtod, and unknown
    /// characters are silently ignored exactly as unmatched C switches
    /// are. The derived-field tail of C's parsecommandline is NOT
    /// applied here; validate() applies it (see the module doc). The
    /// only parse error is the -U switch without the acute feature:
    /// this build mirrors the NO_ACUTE switch rejection (ADR 0004
    /// section 2).
    pub fn parse(options: &str) -> Result<Options, Error> {
        let mut b = Options::new();
        let s = options.as_bytes();
        let mut i = 0usize;
        while i < s.len() {
            let c = s[i];
            if c == b'p' {
                b.poly = true;
            }
            if c == b'r' {
                b.refine = true;
            }
            if c == b'q' {
                b.quality = true;
                if is_num_start(next(s, i)) {
                    b.minangle = take_number(s, &mut i);
                } else {
                    b.minangle = 20.0;
                }
            }
            #[cfg(feature = "acute")]
            if c == b'U' {
                b.quality = true;
                if is_num_start(next(s, i)) {
                    b.maxangle = take_number(s, &mut i);
                } else {
                    b.maxangle = 140.0;
                }
            }
            #[cfg(not(feature = "acute"))]
            if c == b'U' {
                // ADR 0004 section 2: without the acute feature a -U
                // switch must fail exactly as the C NO_ACUTE build
                // fails it (switch rejection).
                return Err(Error::Options(
                    "-U (aCute maximum angle) is not compiled into this build; rebuild with the acute feature".to_string(),
                ));
            }
            if c == b'a' {
                b.quality = true;
                if is_num_start(next(s, i)) {
                    b.fixedarea = true;
                    b.maxarea = take_number(s, &mut i);
                } else {
                    b.vararea = true;
                }
            }
            if c == b'u' {
                b.quality = true;
                b.usertest = true;
            }
            if c == b'A' {
                b.regionattrib = true;
            }
            if c == b'c' {
                b.convex = true;
            }
            if c == b'w' {
                b.weighted = 1;
            }
            if c == b'W' {
                b.weighted = 2;
            }
            if c == b'j' {
                b.jettison = true;
            }
            if c == b'z' {
                b.firstnumber = 0;
            }
            if c == b'n' {
                b.neighbors = true;
            }
            if c == b'B' {
                b.nobound = true;
            }
            if c == b'O' {
                b.noholes = true;
            }
            if c == b'X' {
                b.noexact = true;
            }
            if c == b'o' && next(s, i) == Some(b'2') {
                i += 1;
                b.order = 2;
            }
            if c == b'Y' {
                b.nobisect += 1;
            }
            if c == b'S' {
                b.steiner = 0;
                while let Some(d) = digit(next(s, i)) {
                    i += 1;
                    // C: steiner = steiner * 10 + digit; int overflow in
                    // C is undefined, wrapping keeps the low bits.
                    b.steiner = b.steiner.wrapping_mul(10).wrapping_add(d);
                }
            }
            if c == b'i' {
                b.incremental = true;
            }
            if c == b'F' {
                b.sweepline = true;
            }
            if c == b'l' {
                b.dwyer = false;
            }
            if c == b's' {
                b.splitseg = true;
            }
            if c == b'D' || c == b'L' {
                b.quality = true;
                b.conformdel = true;
            }
            i += 1;
        }
        Ok(b)
    }

    /// The entry-point normalization (ADR 0004 section 5.1): the
    /// transliterated 'behavior_update' (triangle.c:275-303), then the
    /// transliterated 'triangle_check_behavior'
    /// (triangle_helper.c:17-23). Consumes the Options and returns the
    /// normalized internal 'Behavior' the core functions take exactly
    /// where C takes a 'behavior*'. Called by every entry point.
    pub(crate) fn validate(self) -> Result<Behavior, Error> {
        let mut b = self.behavior();
        b.update_derived_fields();
        if b.fixedarea && b.maxarea <= 0.0 {
            // TRI_OPTIONS (triangle_helper.c:17-23).
            return Err(Error::Options(format!(
                "an element area constraint (-aN) requires a positive area, got {}",
                b.maxarea
            )));
        }
        Ok(b)
    }

    fn behavior(self) -> Behavior {
        Behavior {
            poly: self.poly,
            refine: self.refine,
            quality: self.quality,
            vararea: self.vararea,
            fixedarea: self.fixedarea,
            usertest: self.usertest,
            regionattrib: self.regionattrib,
            convex: self.convex,
            weighted: self.weighted,
            jettison: self.jettison,
            firstnumber: self.firstnumber,
            neighbors: self.neighbors,
            nobound: self.nobound,
            noholes: self.noholes,
            noexact: self.noexact,
            conformdel: self.conformdel,
            incremental: self.incremental,
            sweepline: self.sweepline,
            dwyer: self.dwyer,
            splitseg: self.splitseg,
            usesegments: false, // behavior_update derives it below
            order: self.order,
            nobisect: self.nobisect,
            steiner: self.steiner,
            minangle: self.minangle,
            goodangle: 0.0, // behavior_update computes it
            offconstant: 0.0,
            maxarea: self.maxarea,
            #[cfg(feature = "acute")]
            maxangle: self.maxangle,
            #[cfg(feature = "acute")]
            maxgoodangle: 0.0, // behavior_update computes it
            // D13: the RefCell wrap of the callback (Behavior-side only;
            // the Options shape of D13/D9 stays Option<Box<dyn Unsuitable>>).
            user_func: self.user_func.map(std::cell::RefCell::new),
        }
    }

    // -- builder setters, one per switch family (ADR 0004 section 5.1):
    // plain setters, the switch letter documented on each, the C field
    // in the doc. No getters: the inspectable form of record is the
    // options string. -

    /// -p: triangulate a Planar Straight Line Graph. C b->poly = 1.
    pub fn pslg(&mut self) -> &mut Options {
        self.poly = true;
        self
    }
    /// -r: refine a previously generated mesh. C b->refine = 1.
    pub fn refine(&mut self) -> &mut Options {
        self.refine = true;
        self
    }
    /// -q: quality mesh generation. C b->quality = 1.
    pub fn quality(&mut self) -> &mut Options {
        self.quality = true;
        self
    }
    /// -qN: minimum angle bound in degrees (also sets -q).
    /// C b->minangle.
    pub fn min_angle(&mut self, degrees: f64) -> &mut Options {
        self.quality = true;
        self.minangle = degrees;
        self
    }
    /// -UN: maximum angle bound in degrees, the aCute refinement (acute
    /// feature only; also sets -q). C b->maxangle, under the C
    /// #ifndef NO_ACUTE.
    #[cfg(feature = "acute")]
    pub fn max_angle(&mut self, degrees: f64) -> &mut Options {
        self.quality = true;
        self.maxangle = degrees;
        self
    }
    /// -a without a number: apply the input's per-triangle area
    /// constraints. C b->vararea = 1.
    pub fn variable_area(&mut self) -> &mut Options {
        self.quality = true;
        self.vararea = true;
        self
    }
    /// -aN: global maximum triangle area (also sets -q). C
    /// b->fixedarea = 1, b->maxarea = N. validate() rejects a
    /// non-positive area (TRI_OPTIONS).
    pub fn fixed_area(&mut self, max: f64) -> &mut Options {
        self.quality = true;
        self.fixedarea = true;
        self.maxarea = max;
        self
    }
    /// -u: user-defined triangle test via the Unsuitable callback
    /// (also sets -q). C b->usertest = 1, b->triunsuitable_user_func.
    pub fn user_test(&mut self, cb: Box<dyn Unsuitable>) -> &mut Options {
        self.quality = true;
        self.usertest = true;
        self.user_func = Some(cb);
        self
    }
    /// -A: apply attributes to identify triangles in certain regions.
    /// C b->regionattrib = 1.
    pub fn region_attributes(&mut self) -> &mut Options {
        self.regionattrib = true;
        self
    }
    /// -c: enclose the convex hull with segments. C b->convex = 1.
    pub fn convex_hull(&mut self) -> &mut Options {
        self.convex = true;
        self
    }
    /// -w: weighted Delaunay triangulation. C b->weighted = 1.
    pub fn weighted(&mut self) -> &mut Options {
        self.weighted = 1;
        self
    }
    /// -W: regular triangulation, the lower hull of a height field.
    /// C b->weighted = 2.
    pub fn regular(&mut self) -> &mut Options {
        self.weighted = 2;
        self
    }
    /// -j: jettison unused vertices from the output. C b->jettison = 1.
    pub fn jettison(&mut self) -> &mut Options {
        self.jettison = true;
        self
    }
    /// -z: number all items from zero. C b->firstnumber = 0.
    pub fn zero_based(&mut self) -> &mut Options {
        self.firstnumber = 0;
        self
    }
    /// -n: generate a list of triangle neighbors. C b->neighbors = 1.
    pub fn neighbors(&mut self) -> &mut Options {
        self.neighbors = true;
        self
    }
    /// -B: suppress boundary information in output. C b->nobound = 1.
    pub fn no_bounds(&mut self) -> &mut Options {
        self.nobound = true;
        self
    }
    /// -O: ignore holes. C b->noholes = 1.
    pub fn ignore_holes(&mut self) -> &mut Options {
        self.noholes = true;
        self
    }
    /// -X: suppress use of exact arithmetic. C b->noexact = 1.
    pub fn no_exact(&mut self) -> &mut Options {
        self.noexact = true;
        self
    }
    /// -o2: quadratic elements. C b->order (the only value the C loop
    /// accepts after -o is 2).
    pub fn element_order(&mut self, order: i32) -> &mut Options {
        self.order = order;
        self
    }
    /// -Y: suppress boundary segment splitting; the C loop increments
    /// the level per occurrence, so the level is explicit here.
    /// C b->nobisect.
    pub fn no_bisect(&mut self, level: i32) -> &mut Options {
        self.nobisect = level;
        self
    }
    /// -S N: maximum number of added Steiner points. C b->steiner
    /// (0 = none, -1 = unlimited, the default).
    pub fn steiner_limit(&mut self, n: i32) -> &mut Options {
        self.steiner = n;
        self
    }
    /// -i: incremental triangulation method. C b->incremental = 1.
    pub fn incremental(&mut self) -> &mut Options {
        self.incremental = true;
        self
    }
    /// -F: Fortune's sweepline algorithm. C b->sweepline = 1.
    pub fn sweepline(&mut self) -> &mut Options {
        self.sweepline = true;
        self
    }
    /// -l: no alternating cuts (Dwyer's divide-and-conquer heuristic
    /// off; it is on by default). C b->dwyer = 0.
    pub fn no_alternating_cuts(&mut self) -> &mut Options {
        self.dwyer = false;
        self
    }
    /// -s: force segments into the mesh by splitting instead of a CDT.
    /// C b->splitseg = 1.
    pub fn split_segments(&mut self) -> &mut Options {
        self.splitseg = true;
        self
    }
    /// -D / -L: conforming Delaunay, all triangles truly Delaunay
    /// (also sets -q). C b->conformdel = 1.
    pub fn conforming_delaunay(&mut self) -> &mut Options {
        self.quality = true;
        self.conformdel = true;
        self
    }
}

impl std::fmt::Debug for Options {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Options")
            .field("poly", &self.poly)
            .field("refine", &self.refine)
            .field("quality", &self.quality)
            .field("vararea", &self.vararea)
            .field("fixedarea", &self.fixedarea)
            .field("usertest", &self.usertest)
            .field("regionattrib", &self.regionattrib)
            .field("convex", &self.convex)
            .field("weighted", &self.weighted)
            .field("jettison", &self.jettison)
            .field("firstnumber", &self.firstnumber)
            .field("neighbors", &self.neighbors)
            .field("nobound", &self.nobound)
            .field("noholes", &self.noholes)
            .field("noexact", &self.noexact)
            .field("conformdel", &self.conformdel)
            .field("incremental", &self.incremental)
            .field("sweepline", &self.sweepline)
            .field("dwyer", &self.dwyer)
            .field("splitseg", &self.splitseg)
            .field("order", &self.order)
            .field("nobisect", &self.nobisect)
            .field("steiner", &self.steiner)
            .field("minangle", &self.minangle)
            .field("maxarea", &self.maxarea)
            .field("maxangle", &self.maxangle_field())
            .field("user_func", &self.user_func.as_ref().map(|_| "<callback>"))
            .finish()
    }
}

impl Options {
    /// The -UN bound, present only when the acute feature is compiled
    /// in (the C #ifndef NO_ACUTE field mirror; ADR 0004 section 2).
    #[cfg(feature = "acute")]
    fn maxangle_field(&self) -> Option<f64> {
        Some(self.maxangle)
    }

    #[cfg(not(feature = "acute"))]
    fn maxangle_field(&self) -> Option<f64> {
        None
    }
}

impl Default for Options {
    fn default() -> Options {
        Options::new()
    }
}

/// The normalized internal form of the options, the exact field-mirror
/// of C's 'struct behavior' (triangle.h:371-470) including the derived
/// fields. The transliterated core takes '&mut Behavior' exactly where
/// C takes 'behavior *b'. Produced by Options::validate()
/// (behavior_update + triangle_check_behavior); stage-1 core and tests
/// may also assemble it directly to mirror C's internal call shapes.
pub(crate) struct Behavior {
    pub(crate) poly: bool,
    pub(crate) refine: bool,
    pub(crate) quality: bool,
    pub(crate) vararea: bool,
    pub(crate) fixedarea: bool,
    pub(crate) usertest: bool,
    pub(crate) regionattrib: bool,
    pub(crate) convex: bool,
    pub(crate) weighted: i32,
    pub(crate) jettison: bool,
    pub(crate) firstnumber: i32,
    pub(crate) neighbors: bool,
    pub(crate) nobound: bool,
    pub(crate) noholes: bool,
    pub(crate) noexact: bool,
    pub(crate) conformdel: bool,
    pub(crate) incremental: bool,
    pub(crate) sweepline: bool,
    pub(crate) dwyer: bool,
    pub(crate) splitseg: bool,
    pub(crate) usesegments: bool,
    pub(crate) order: i32,
    pub(crate) nobisect: i32,
    pub(crate) steiner: i32,
    pub(crate) minangle: f64,
    pub(crate) goodangle: f64,
    pub(crate) offconstant: f64,
    pub(crate) maxarea: f64,
    #[cfg(feature = "acute")]
    pub(crate) maxangle: f64,
    #[cfg(feature = "acute")]
    pub(crate) maxgoodangle: f64,
    /// The -u sizing callback (C b->triunsuitable_user_func, invoked at
    /// testtriangle). Deviation D13 (docs/deviation-register.md): the
    /// crate-private Behavior stores the box in a RefCell so the
    /// transliterated core's `&Behavior` plumbing (C's `behavior*`
    /// read-flow) can invoke the `&mut self` object-safe method inside
    /// testtriangle; the public Options storage of D9
    /// (`Option<Box<dyn Unsuitable>>`) is unchanged, and the MT
    /// milestone (ADR 0005, #12) supersedes the interior shape when the
    /// storage bound becomes Send + Sync.
    pub(crate) user_func: Option<std::cell::RefCell<Box<dyn Unsuitable>>>,
}

impl Behavior {
    /// The 'Options::new()' equivalent in Behavior form: the
    /// 'parsecommandline("")' defaults, pre-derived-tail (goodangle/
    /// offconstant are the 0.0 placeholders behavior_update fills;
    /// C computes them in the parsecommandline tail instead). Tests
    /// assembling a Behavior mirror C defaulting a behavior struct
    /// before its update.
    pub(crate) fn default_form() -> Behavior {
        Options::new().behavior()
    }

    /// C behavior_update (triangle.c:275-303), the identical tail C's
    /// parsecommandline runs (triangle.c:527-552). PI is the C literal
    /// of triangle_config.h:159 (the double truncation of both
    /// literals is identical); cos/sqrt call the same system libm on
    /// both sides of the differential run (ADR 0004 section 3).
    pub(crate) fn update_derived_fields(&mut self) {
        self.usesegments = self.poly || self.refine || self.quality || self.convex;
        self.goodangle = (self.minangle * PI / 180.0).cos();
        #[cfg(feature = "acute")]
        {
            self.maxgoodangle = (self.maxangle * PI / 180.0).cos();
        }
        if self.goodangle == 1.0 {
            self.offconstant = 0.0;
        } else {
            self.offconstant = 0.475 * ((1.0 + self.goodangle) / (1.0 - self.goodangle)).sqrt();
        }
        self.goodangle *= self.goodangle;
        // No element area constraints will ever be assigned unless a
        // PSLG or a refinement supplies them (C comment, triangle.c:288-289).
        if !self.refine && !self.poly {
            self.vararea = false;
        }
        // No extra attribute per element unless the input supports it
        // (C comment, triangle.c:293-294).
        if self.refine || !self.poly {
            self.regionattrib = false;
        }
        // Regular/weighted triangulations are incompatible with PSLGs
        // and meshing (C comment, triangle.c:298-299).
        if self.weighted != 0 && (self.poly || self.quality) {
            self.weighted = 0;
        }
    }
}

/// C peeks options[i + 1] (the NUL ends the run); None is the NUL.
fn next(s: &[u8], i: usize) -> Option<u8> {
    s.get(i + 1).copied()
}

/// The digit/dot test of C's number branches.
fn is_num_start(c: Option<u8>) -> bool {
    match c {
        Some(b'.') => true,
        Some(d) if d.is_ascii_digit() => true,
        _ => false,
    }
}

/// The digit test of C's -S branch.
fn digit(c: Option<u8>) -> Option<i32> {
    match c {
        Some(d) if d.is_ascii_digit() => Some((d - b'0') as i32),
        _ => None,
    }
}

/// The inline number parsing of -q/-U/-a: C collects the digit/dot run
/// into workstring and hands it to strtod. This mirrors strtod's
/// longest-valid-prefix semantics over the digits/dots alphabet
/// ("1.2.3" parses as 1.2, "5." as 5, "." as 0.0: no conversion).
fn take_number(s: &[u8], i: &mut usize) -> f64 {
    let start = *i + 1;
    let mut end = start;
    while end < s.len() && (s[end] == b'.' || s[end].is_ascii_digit()) {
        end += 1;
    }
    *i = end - 1;
    strtod_prefix(&s[start..end])
}

fn strtod_prefix(bytes: &[u8]) -> f64 {
    for len in (1..=bytes.len()).rev() {
        if let Ok(txt) = std::str::from_utf8(&bytes[..len]) {
            if let Ok(v) = txt.parse::<f64>() {
                return v;
            }
        }
    }
    0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bit-exact on native runs (the gate that pins the formulas);
    /// tolerance-checked under miri, whose transcendental shims carry
    /// deliberate host-rounding noise (miri stays advisory).
    fn assert_exact_or_miri_close(actual: f64, expected: f64) {
        if cfg!(miri) {
            assert!(
                (actual - expected).abs() < 1e-12,
                "actual {} expected {}",
                actual,
                expected
            );
        } else {
            assert_eq!(actual, expected);
        }
    }

    fn bh(s: &str) -> Behavior {
        Options::parse(s)
            .expect("parse")
            .validate()
            .expect("validate")
    }

    #[test]
    fn defaults_are_the_parsecommandline_empty_run() {
        let b = bh("");
        assert!(!b.poly);
        assert!(!b.refine);
        assert!(!b.quality);
        assert_eq!(b.firstnumber, 1);
        assert!(b.dwyer);
        assert_eq!(b.steiner, -1);
        assert_eq!(b.order, 1);
        assert_eq!(b.nobisect, 0);
        assert_eq!(b.minangle, 0.0);
        assert_eq!(b.maxarea, -1.0);
        assert!(!b.usesegments);
        assert_eq!(b.weighted, 0);
        #[cfg(feature = "acute")]
        assert_eq!(b.maxangle, 0.0);
    }

    #[test]
    fn new_equals_parse_empty() {
        let a = Options::new().validate().expect("validate");
        let b = bh("");
        assert_eq!(a.poly, b.poly);
        assert_eq!(a.dwyer, b.dwyer);
        assert_eq!(a.steiner, b.steiner);
        assert_eq!(a.order, b.order);
        assert_eq!(a.maxarea, b.maxarea);
    }

    #[test]
    fn q_number_parsing_and_strtod_semantics() {
        assert_eq!(bh("").minangle, 0.0);
        assert_eq!(bh("q").minangle, 20.0);
        assert_eq!(bh("q35").minangle, 35.0);
        assert_eq!(bh("q35.5").minangle, 35.5);
        // strtod prefix semantics on digit/dot runs.
        assert_eq!(bh("q1.2.3").minangle, 1.2);
        assert_eq!(bh("q5.").minangle, 5.0);
        assert_eq!(bh("q.").minangle, 0.0);
        assert_eq!(bh("q..7").minangle, 0.0);
        assert!(bh("q").quality);
        assert!(bh("q").usesegments);
    }

    #[test]
    fn usesegments_derivation() {
        assert!(!bh("").usesegments);
        assert!(bh("p").usesegments);
        assert!(bh("r").usesegments);
        assert!(bh("q").usesegments);
        assert!(bh("c").usesegments);
        assert!(!bh("i").usesegments);
        assert!(!bh("z").usesegments);
    }

    #[test]
    fn vararea_and_its_correction() {
        // -a without poly/refine is corrected away (triangle.c:290-292).
        assert!(!bh("a").vararea);
        assert!(!bh("a").fixedarea);
        assert_eq!(bh("a").maxarea, -1.0);
        // With -p it survives.
        assert!(bh("pa").vararea);
        // -aN sets fixedarea and maxarea.
        let b = bh("a2.5");
        assert!(b.fixedarea);
        assert_eq!(b.maxarea, 2.5);
    }

    #[test]
    fn fixedarea_validation_mirrors_check_behavior() {
        // triangle_check_behavior: fixedarea && maxarea <= 0 is
        // TRI_OPTIONS.
        assert!(Options::parse("a0").unwrap().validate().is_err());
        assert!(Options::parse("a0.0").unwrap().validate().is_err());
        let b = bh("a1.0");
        assert_eq!(b.maxarea, 1.0);
    }

    #[test]
    fn regionattrib_correction() {
        // No extra element attribute unless a PSLG (not refine) is read
        // (triangle.c:295-297).
        assert!(!bh("A").regionattrib);
        assert!(bh("pA").regionattrib);
        // refine also cancels it.
        assert!(!bh("prA").regionattrib);
    }

    #[test]
    fn weighted_correction() {
        assert_eq!(bh("w").weighted, 1);
        assert_eq!(bh("W").weighted, 2);
        // Incompatible with PSLG and quality (triangle.c:300-302).
        assert_eq!(bh("pw").weighted, 0);
        assert_eq!(bh("qW").weighted, 0);
        assert_eq!(bh("wW").weighted, 2);
    }

    #[test]
    fn steiner_nobisect_order() {
        assert_eq!(bh("S").steiner, 0);
        assert_eq!(bh("S100").steiner, 100);
        assert_eq!(bh("Y").nobisect, 1);
        assert_eq!(bh("YY").nobisect, 2);
        assert_eq!(bh("o2").order, 2);
        // Unknown switches are silently ignored, C-style.
        assert_eq!(bh("o").order, 1);
        assert_eq!(bh("vxy").order, 1);
        assert!(!bh("vxy").quality);
    }

    #[test]
    fn derived_angle_math_is_bit_exact() {
        // bh("") runs minangle 0: cos(0) = 1, so offconstant is 0 and
        // goodangle is squared 1.
        let b = bh("");
        assert_eq!(b.goodangle, 1.0);
        assert_eq!(b.offconstant, 0.0);
        // The 20-degree default of a bare -q. black_box keeps the
        // expected value on the same runtime-libm path as
        // behavior_update (const-folded cos differs under miri).
        let b = bh("q");
        let g = (std::hint::black_box(20.0f64) * PI / 180.0).cos();
        assert_exact_or_miri_close(b.goodangle, g * g);
        assert_exact_or_miri_close(b.offconstant, 0.475 * ((1.0 + g) / (1.0 - g)).sqrt());
        assert_eq!(b.minangle, 20.0);
    }

    #[test]
    fn switch_flags_roundup() {
        let b = bh("jzBOnXcsilDFD");
        assert!(b.jettison);
        assert_eq!(b.firstnumber, 0);
        assert!(b.nobound);
        assert!(b.noholes);
        assert!(b.noexact);
        assert!(b.convex);
        assert!(b.splitseg);
        assert!(b.incremental);
        assert!(b.sweepline);
        assert!(b.conformdel);
        assert!(b.quality);
        assert!(!b.dwyer);
        let b = bh("l");
        assert!(!b.dwyer);
    }

    #[cfg(feature = "acute")]
    #[test]
    fn acute_switches() {
        let b = bh("U");
        assert!(b.quality);
        assert_eq!(b.maxangle, 140.0);
        let expected = (std::hint::black_box(140.0f64) * PI / 180.0).cos();
        assert_exact_or_miri_close(b.maxgoodangle, expected);
        let b = bh("U160.5");
        assert_eq!(b.maxangle, 160.5);
    }

    #[cfg(not(feature = "acute"))]
    #[test]
    fn u_switch_is_rejected_without_the_acute_feature() {
        assert!(matches!(Options::parse("U"), Err(Error::Options(_))));
        assert!(matches!(Options::parse("pU140"), Err(Error::Options(_))));
        // Everything else still parses.
        assert!(Options::parse("pq30").is_ok());
    }

    struct AlwaysUnsuitable;

    impl Unsuitable for AlwaysUnsuitable {
        fn is_unsuitable(
            &mut self,
            _org: VertexSample,
            _dest: VertexSample,
            _apex: VertexSample,
            _area: f64,
        ) -> bool {
            true
        }
    }

    #[test]
    fn user_test_callback_rides_the_behavior() {
        let mut opts = Options::new();
        opts.user_test(Box::new(AlwaysUnsuitable));
        let b = opts.validate().expect("validate");
        assert!(b.usertest);
        assert!(b.quality); // -u sets it
                            // D13: the Behavior-side RefCell wrap carries the stored box.
        let cb = b.user_func.as_ref().expect("callback stored");
        let s = VertexSample {
            x: 0.0,
            y: 0.0,
            marker: 7,
        };
        assert!(cb.borrow_mut().is_unsuitable(s, s, s, 1.5));
    }

    #[test]
    fn builder_mirrors_parse() {
        let mut opts = Options::new();
        opts.pslg()
            .quality()
            .zero_based()
            .neighbors()
            .steiner_limit(10);
        let b = opts.validate().expect("validate");
        let p = bh("pqznS10");
        assert_eq!(b.poly, p.poly);
        assert_eq!(b.quality, p.quality);
        assert_eq!(b.firstnumber, p.firstnumber);
        assert_eq!(b.neighbors, p.neighbors);
        assert_eq!(b.steiner, p.steiner);
        assert_eq!(b.usesegments, p.usesegments);
    }
}
