// vfmt.rs — number and string FORMATTING, in one place, because the emitted text is a contract
// everywhere it is used: the committed URDF `mjcf_convert` writes, the recorder document (the
// player's wire format), and the bench/probe JSON compared against pinned text. One rule, one place:
// these were once copied per program and had already drifted (see the per-function notes).

/// f64_str renders one float in the pinned form, over Ryu: the two signed zeros spelled out, decimal
/// inside [1e-4, 1e6) and scientific outside it, the shortest digits that round-trip, `.0` kept on an
/// integral value, and an exact tie rounded to EVEN. Rust's `Display` does none of those three.
pub fn f64_str(f: f64) -> String {
    if f == 0.0 {
        if f.is_sign_negative() {
            return "-0.0".to_string();
        }
        return "0.0".to_string();
    }
    let (digits, exp) = shortest_digits(f);
    if f.abs() >= 1e-4 && f.abs() < 1e6 {
        decimal_form(f < 0.0, &digits, exp)
    } else {
        scientific_form(f < 0.0, &digits, exp)
    }
}

/// f32_str is the pinned text for one f32 over Ryu's SINGLE-precision path: its digits are the f32's
/// own shortest, NOT the f32 widened to f64 (`9.9998f32` renders `9.9998`, not `9.999799728393555`).
pub fn f32_str(f: f32) -> String {
    if f == 0.0 {
        if f.is_sign_negative() {
            return "-0.0".to_string();
        }
        return "0.0".to_string();
    }
    if f.abs() >= 1e-4 && f.abs() < 1e6 {
        let d = format!("{f}");
        if d.contains('.') {
            return d;
        }
        return d + ".0";
    }
    let sci = format!("{f:e}");
    match sci.split_once('e') {
        Some((mant, exp)) => {
            let (sign, digits) = match exp.strip_prefix('-') {
                Some(d) => ('-', d),
                None => ('+', exp),
            };
            format!("{mant}e{sign}{digits:0>2}")
        }
        // no exponent: a non-finite value (nan/inf in the pinned text)
        None => sci,
    }
}

/// f32_arr_str is `arr_str`'s single-precision twin, each element through `f32_str`.
pub fn f32_arr_str(xs: &[f32]) -> String {
    let parts: Vec<String> = xs.iter().map(|x| f32_str(*x)).collect();
    format!("[{}]", parts.join(", "))
}

/// arr_str renders a float slice as `[a, b, c]`, each element through `f64_str`.
pub fn arr_str(xs: &[f64]) -> String {
    let parts: Vec<String> = xs.iter().map(|x| f64_str(*x)).collect();
    format!("[{}]", parts.join(", "))
}

/// c_exp renders C's `%e`: six fractional digits, a signed exponent of at least two digits.
pub fn c_exp(x: f64) -> String {
    c_exp_prec(x, 6)
}

/// c_exp_prec is `c_exp` with the precision written out: `.Ne` is C's `%.Ne`, N the fraction width
/// exactly (no off-by-one the way `dec`'s bare form has).
pub fn c_exp_prec(x: f64, prec: usize) -> String {
    let sci = format!("{x:.prec$e}");
    let (mant, exp) = sci
        .split_once('e')
        .expect("a fixed-precision form has an exponent");
    let (sign, mag) = match exp.strip_prefix('-') {
        Some(d) => ('-', d),
        None => ('+', exp),
    };
    format!("{mant}e{sign}{mag:0>2}")
}

/// dec renders the BARE `.N` precision form, which carries N-1 decimal places, NOT N: `.12` renders
/// eleven (1203 values of the committed G1 URDF confirm it) and `.6` renders five
/// (`0.0039635 -> 0.00396`). The `f`-suffixed form is C's and renders N, hence `fixed_6` below.
pub fn dec(x: f64, v_digits: usize) -> String {
    if v_digits == 0 {
        // the `.0` form never appears in this tree; spelled out so the subtraction below cannot underflow.
        return format!("{x:.0}");
    }
    format!("{x:.prec$}", prec = v_digits - 1)
}

/// fixed_6 renders `'${x:.6f}'`: six decimal places, the `f`-suffixed form — NOT `dec(x, 6)`.
pub fn fixed_6(x: f64) -> String {
    format!("{x:.6}")
}

/// json_number renders one float in the JSON form: `f64_str` with a trailing ".0" stripped, so an
/// integral value is `1` in a JSON document and `1.0` in a printout.
pub fn json_number(f: f64) -> String {
    let mut s = f64_str(f);
    if s.len() > 2 && s.ends_with(".0") {
        s.truncate(s.len() - 2);
    }
    s
}

/// json_array renders a float vector in the JSON form: no space after the commas, each element
/// through `json_number` — the JSON layout, NOT `arr_str`'s `[a, b, c]` one (the encoder writes
/// `[1,2]` where the float-slice layout writes `[1.0, 2.0]`).
pub fn json_array(xs: &[f64]) -> String {
    let parts: Vec<String> = xs.iter().map(|x| json_number(*x)).collect();
    format!("[{}]", parts.join(","))
}

/// json_string quotes and escapes a string for a JSON document (the subset `json2.encode` uses
/// for the labels and paths these programs write).
pub fn json_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            _ => out.push(c),
        }
    }
    out.push('"');
    out
}

/// shortest_digits returns the shortest digit string that parses back to f and the exponent of its
/// leading digit (d0.d1d2... * 10^exp). Rust's `{:.*e}` rounds an exact tie to the EVEN digit — the
/// pinned rounding — so probing precisions upward yields the shortest digits.
fn shortest_digits(f: f64) -> (String, i32) {
    let mut s = format!("{f:.16e}");
    for p in 1..=17 {
        let c = format!("{:.*e}", p - 1, f);
        if let Ok(v) = c.parse::<f64>() {
            if v == f {
                s = c;
                break;
            }
        }
    }
    let (mant, exp) = s
        .split_once('e')
        .expect("a fixed-precision form has an exponent");
    let digits: String = mant.chars().filter(char::is_ascii_digit).collect();
    (
        digits,
        exp.parse::<i32>().expect("a one- or two-digit exponent"),
    )
}

/// decimal_form places the point in the pinned decimal notation: no exponent, and a `.0` when every digit is integer part.
fn decimal_form(neg: bool, digits: &str, exp: i32) -> String {
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if exp < 0 {
        out.push_str("0.");
        for _ in 0..(-exp - 1) {
            out.push('0');
        }
        out.push_str(digits);
    } else {
        let ip = (exp + 1) as usize;
        if ip >= digits.len() {
            out.push_str(digits);
            for _ in digits.len()..ip {
                out.push('0');
            }
            out.push_str(".0");
        } else {
            out.push_str(&digits[..ip]);
            out.push('.');
            out.push_str(&digits[ip..]);
        }
    }
    out
}

/// scientific_form is the pinned scientific notation: one digit, then the rest with trailing zeros
/// dropped and no `.0` when there is no fraction, and an exponent that always carries its sign and
/// at least two digits (`1e+06`, `6.103515625e-05`).
fn scientific_form(neg: bool, digits: &str, exp: i32) -> String {
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    out.push_str(&digits[..1]);
    let frac = digits[1..].trim_end_matches('0');
    if !frac.is_empty() {
        out.push('.');
        out.push_str(frac);
    }
    let (sign, mag) = if exp < 0 { ('-', -exp) } else { ('+', exp) };
    out.push('e');
    out.push(sign);
    out.push_str(&format!("{mag:02}"));
    out
}
