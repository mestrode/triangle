//! The C printf formats of the I/O phase (the formatter seam): the
//! '%.17g' family of the file writers (triangle_io.c:51, 55, 61, 104,
//! 117, 162-186, 237-253, 303, 178-190), the '%.3g' of the EPS writer
//! (eps_writer.c:194, 200-201) and the width-padded g-forms of the
//! example CLI's renderers (src/examples/triangle-cli/util.c:353-374:
//! %16.5g, %12.5g, %8.5g, %6.6g, %15.5g).
//! std-only: no libc at the boundary, no crates.
//!
//! Correctness argument: every decimal form is derived from Rust's
//! correctly-rounded '{:.16e}' scientific rendering (std renders the
//! exact binary value with correct rounding, exactly like glibc's
//! printf), so the bytes match C on every input. The g-form picks
//! e- or f-form by C's rule (decimal exponent < -4 or >= precision
//! -> e-form), strips trailing zeros and a bare trailing point, and
//! writes the exponent with at least two digits ('e-05', never
//! 'e-5'). The width forms are the g-form space-padded to the field
//! width (right-aligned, or left-aligned for the '-' flag), C's
//! right-alignment default included. Everything is pinned by
//! construction in the tests below, never via the C oracle.
//!
//! This module is public because the example-CLI renderers
//! (print_statistics) live in the bin crate and render the public
//! Statistics/Quality types (ADR 0004 section 6: the CLI's verbose
//! text reports are bin-crate rendering of these types, pinned in
//! the I/O phase), and the g-form is exactly C's printf behavior they
//! must reproduce.

/// '%.*g' of C printf (glibc), measured at the writer sites above: at
/// most `precision` significant digits, e-form iff the normalized
/// decimal exponent is < -4 or >= precision, trailing zeros and a bare
/// trailing point stripped, exponent with at least two digits.
/// nan/inf and -0 render like glibc ('nan', '-nan', 'inf', '-inf',
/// '-0').
pub fn g(v: f64, precision: usize) -> String {
    if v.is_nan() {
        // glibc renders the sign of a negative NaN.
        return if v.is_sign_negative() {
            "-nan".to_string()
        } else {
            "nan".to_string()
        };
    }
    if v.is_infinite() {
        return if v < 0.0 {
            "-inf".to_string()
        } else {
            "inf".to_string()
        };
    }
    let neg = v.is_sign_negative();
    let a = v.abs();
    if a == 0.0 {
        // '%.17g' of 0.0 is "0", of -0.0 is "-0".
        return if neg {
            "-0".to_string()
        } else {
            "0".to_string()
        };
    }
    // C: precision 0 is treated as 1.
    let p = precision.max(1);
    // The correctly-rounded scientific form: one digit before the
    // point, p - 1 after it ('{:.16e}' for the writers' %.17g).
    let sci = format!("{:.*e}", p - 1, a);
    let epos = sci.find('e').expect("scientific form");
    let mantissa = &sci[..epos];
    let exp: i64 = sci[epos + 1..].parse().expect("exponent");
    // The p significant digits, point removed (the first is never
    // '0' for a nonzero value).
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();

    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if exp < -4 || exp >= p as i64 {
        // The e-form: d[.ddd]e[+-]XX with trailing zeros stripped.
        out.push_str(&digits[..1]);
        let frac = strip_trailing_zeros(&digits[1..]);
        if !frac.is_empty() {
            out.push('.');
            out.push_str(frac);
        }
        out.push('e');
        out.push(if exp < 0 { '-' } else { '+' });
        // At least two exponent digits (C: 'e-05', not 'e-5').
        let mag = exp.unsigned_abs();
        if mag < 10 {
            out.push('0');
        }
        out.push_str(&mag.to_string());
    } else if exp >= 0 {
        // The f-form with an integer part: digits[0..=exp].
        let intlen = (exp + 1) as usize;
        out.push_str(&digits[..intlen]);
        let frac = strip_trailing_zeros(&digits[intlen..]);
        if !frac.is_empty() {
            out.push('.');
            out.push_str(frac);
        }
    } else {
        // The f-form below one: 0.[-4..-1 zeros]digits.
        out.push_str("0.");
        for _ in 0..(-exp - 1) {
            out.push('0');
        }
        out.push_str(strip_trailing_zeros(&digits));
    }
    out
}

/// '%*.*g' with the '-' flag off: the g-form space-padded on the left
/// to `width` (C's right-alignment default), measured at
/// util.c:348-349, 353, 365 (%16.5g, %12.5g, %8.5g, %15.5g, %6.6g).
pub fn g_padded(v: f64, precision: usize, width: usize) -> String {
    let s = g(v, precision);
    let len = s.chars().count();
    if len >= width {
        s
    } else {
        let mut padded = String::new();
        for _ in 0..width - len {
            padded.push(' ');
        }
        padded.push_str(&s);
        padded
    }
}

/// '%-*.*g': the g-form space-padded on the right to `width' (the
/// left-aligned table cells of the aspect-ratio histogram,
/// util.c:352-361: '%-6.6g').
pub fn g_padded_left(v: f64, precision: usize, width: usize) -> String {
    let s = g(v, precision);
    let len = s.chars().count();
    if len >= width {
        s
    } else {
        let mut padded = String::from(&s);
        for _ in 0..width - len {
            padded.push(' ');
        }
        padded
    }
}

/// '%d' of C printf (the plain int forms of the writers).
pub(crate) fn d(v: i32) -> String {
    v.to_string()
}

/// '%ld' of C printf (the plain long forms of the writers: the item
/// counts of the headers).
pub(crate) fn ld(v: i64) -> String {
    v.to_string()
}

/// '%4d' of C printf: the int right-aligned in a field of at least
/// four spaces (the vertex numbers of the .node writer,
/// triangle_io.c:51).
pub(crate) fn d4(v: i32) -> String {
    pad_left(&v.to_string(), 4)
}

/// '%4ld' of C printf: the long right-aligned in a field of at least
/// four spaces (the element, subsegment, edge, neighbor and hole
/// numbers, triangle_io.c:104, 162-186, 237-253, 303, 178-190).
pub(crate) fn ld4(v: i64) -> String {
    pad_left(&v.to_string(), 4)
}

fn pad_left(s: &str, width: usize) -> String {
    let len = s.chars().count();
    if len >= width {
        return s.to_string();
    }
    let mut out = String::new();
    for _ in 0..width - len {
        out.push(' ');
    }
    out.push_str(s);
    out
}

fn strip_trailing_zeros(s: &str) -> &str {
    s.trim_end_matches('0')
}

#[cfg(test)]
mod tests {
    use super::*;

    // The %.17g pins, by construction from the printf spec (the
    // exact binary value, correctly rounded to 17 significant digits,
    // trailing zeros stripped):
    #[test]
    fn g17_half() {
        assert_eq!(g(0.5, 17), "0.5");
    }

    #[test]
    fn g17_one() {
        assert_eq!(g(1.0, 17), "1");
    }

    #[test]
    fn g17_hundred() {
        assert_eq!(g(100.0, 17), "100");
    }

    #[test]
    fn g17_one_fifth() {
        // 0.2 = 0x1.999999999999Ap-3 = 0.2000000000000000111...:
        // 17 significant digits round to ...00011.
        assert_eq!(g(0.2, 17), "0.20000000000000001");
    }

    #[test]
    fn g17_one_tenth() {
        // 0.1 = 0x1.999999999999Ap-4 = 0.1000000000000000055511...:
        // the 17-digit form of the writers' example.
        assert_eq!(g(0.1, 17), "0.10000000000000001");
    }

    #[test]
    fn g17_exponent_boundary_minus_four() {
        // 1e-4: exponent -4 is f-form ('0.0001'); 1e-5: exponent -5
        // is e-form with two exponent digits - and the nearest
        // double, 1.0000000000000000818...e-05, keeps 17 nonzero
        // digits.
        assert_eq!(g(0.0001, 17), "0.0001");
        assert_eq!(g(1e-5, 17), "1.0000000000000001e-05");
        assert_eq!(g(-1e-5, 17), "-1.0000000000000001e-05");
    }

    #[test]
    fn g17_e_form_at_precision_boundary() {
        // 1e16: exponent 16 < 17 is f-form; 1e17: exponent 17 >= 17
        // is e-form.
        assert_eq!(g(1e16, 17), "10000000000000000");
        assert_eq!(g(1e17, 17), "1e+17");
    }

    #[test]
    fn g17_integers_exact() {
        assert_eq!(g(1234567.0, 17), "1234567");
        assert_eq!(g(-42.0, 17), "-42");
    }

    #[test]
    fn g17_zero_and_negative_zero() {
        assert_eq!(g(0.0, 17), "0");
        assert_eq!(g(-0.0, 17), "-0");
    }

    #[test]
    fn g17_infinities_and_nan() {
        assert_eq!(g(f64::INFINITY, 17), "inf");
        assert_eq!(g(f64::NEG_INFINITY, 17), "-inf");
        assert_eq!(g(f64::NAN, 17), "nan");
        assert_eq!(g(-f64::NAN, 17), "-nan");
    }

    #[test]
    fn g17_nonzero_digits_kept() {
        // A value whose 17-digit expansion has no trailing zero: the
        // binary value 0.1 + 0.2 is 0.3000000000000000444089...,
        // 17 digits round to ...00004.
        assert_eq!(g(0.1 + 0.2, 17), "0.30000000000000004");
    }

    // The %.3g pins of the EPS writer (f32 constants promoted to
    // f64 by C's default argument promotion, eps_writer.c:97, 141,
    // 169, 192-203):
    #[test]
    fn g3_eps_colors() {
        assert_eq!(g(0.6f32 as f64, 3), "0.6");
        assert_eq!(g(0.4f32 as f64, 3), "0.4");
        assert_eq!(g(0.0f32 as f64, 3), "0");
        assert_eq!(g(0.27f32 as f64, 3), "0.27");
        assert_eq!(g(0.5f32 as f64, 3), "0.5");
        assert_eq!(g(0.7f32 as f64, 3), "0.7");
        assert_eq!(g(0.9f32 as f64, 3), "0.9");
        assert_eq!(g(0.8f32 as f64, 3), "0.8");
        assert_eq!(g(0.4f32 as f64, 3), "0.4");
    }

    #[test]
    fn g3_e_form_kicks_in_at_1000() {
        // %.3g: exponent >= 3 is e-form ('1e+03' for 1000).
        assert_eq!(g(100.0, 3), "100");
        assert_eq!(g(999.0, 3), "999");
        assert_eq!(g(1000.0, 3), "1e+03");
    }

    // The padded forms of print_statistics (util.c:353-374):
    #[test]
    fn g_padded_forms() {
        // %16.5g of 1.5: right-aligned in 16.
        assert_eq!(g_padded(1.5, 5, 16), "             1.5");
        // %6.6g of 1.5 (the histogram's left column).
        assert_eq!(g_padded(1.5, 6, 6), "   1.5");
        // %-6.6g of 2.0 (left-aligned).
        assert_eq!(g_padded_left(2.0, 6, 6), "2     ");
        // %8.5g of an aspect ratio: %.5g strips the trailing
        // zero before the width pads.
        assert_eq!(g_padded(1.118, 5, 8), "   1.118");
        // No truncation: a wider value keeps its bytes.
        assert_eq!(g_padded(1234567.89, 5, 6), "1.2346e+06");
    }

    // The %4d/%4ld pads of the writers (triangle_io.c:51, 104):
    #[test]
    fn int_field_pads() {
        assert_eq!(d4(1), "   1");
        assert_eq!(d4(123), " 123");
        assert_eq!(d4(1234), "1234");
        assert_eq!(d4(12345), "12345");
        assert_eq!(d4(-1), "  -1");
        assert_eq!(ld4(1), "   1");
        assert_eq!(ld4(-12), " -12");
        assert_eq!(ld4(99999), "99999");
        assert_eq!(d(1), "1");
        assert_eq!(ld(-1), "-1");
    }
}
