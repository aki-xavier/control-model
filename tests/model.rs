use control_math::vec3::Vec3;
use control_model::urdf::rpy_to_r;
use control_model::xml::parse_document;

#[test]
fn xml_reads_a_root_and_rejects_malformed_input() {
    let ok = parse_document("<a><b x=\"1\"/></a>").expect("well-formed");
    assert_eq!(ok.name, "a");
    assert_eq!(ok.children.len(), 1);
    assert_eq!(ok.children[0].name, "b");
    assert_eq!(ok.children[0].attr_or("x", ""), "1");
    assert!(parse_document("<a>").is_err());
}

// Zero rotation is the identity, which is the anchor the rest of the chain arithmetic is built on;
// rpy_to_r is checked against the quaternion path in tests/urdf.rs.
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
