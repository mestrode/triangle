//! The public options surface: Options (the mirror of struct behavior),
//! Options::new / Options::parse (the transliterated parsecommandline
//! defaults and character loop) and validate (the transliterated
//! triangle_check_behavior + derived-field tail: usesegments, goodangle,
//! maxgoodangle, offconstant), plus the internal normalized Behavior
//! consumed by the transliterated core.
//! 
//! ADR 0004 section 5.1; the callback trait Unsuitable/VertexSample
//! (triunsuitable_user_func) is specified in section 5.4.
