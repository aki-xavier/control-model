use control_math::vec3::Vec3;
use control_model::urdf::rpy_to_r;
use control_model::vfmt::{f64_str, json_number};
use control_model::xml::parse_document;

// The formatting is a contract (the committed URDF and the recorder's wire format are compared byte
// for byte), so a few of its pinned forms are asserted here rather than only in simu's suites.
#[test]
fn vfmt_pins_its_own_forms() {
    assert_eq!(f64_str(0.0), "0.0");
    assert_eq!(f64_str(-0.0), "-0.0");
    assert_eq!(f64_str(1.0), "1.0");
    assert_eq!(f64_str(0.001), "0.001");
    assert_eq!(f64_str(1e7), "1e+07");
    assert_eq!(json_number(1.0), "1");
}

#[test]
fn xml_reads_a_root_and_rejects_malformed_input() {
    let ok = parse_document("<a><b x=\"1\"/></a>").expect("well-formed");
    assert_eq!(ok.name, "a");
    assert_eq!(ok.children.len(), 1);
    assert_eq!(ok.children[0].name, "b");
    assert_eq!(ok.children[0].attr_or("x", ""), "1");
    assert!(parse_document("<a>").is_err());
}

// rpy_to_r is the URDF fixed-axis convention; zero rotation is the identity, which is the anchor the
// rest of the chain arithmetic is built on.
#[test]
fn zero_rpy_is_the_identity_rotation() {
    let r = rpy_to_r(&Vec3::ZERO);
    for i in 0..3 {
        for j in 0..3 {
            let want = if i == j { 1.0 } else { 0.0 };
            assert!((r.at(i, j) - want).abs() < 1e-15, "r[{i}][{j}]");
        }
    }
}
