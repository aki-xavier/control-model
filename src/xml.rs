// xml.rs — XmlNode (one element of the parsed tree) and parse_document, the reader for the
// URDF and MJCF documents; the parsing itself is roxmltree's. Deliberately strict: malformed
// XML, an unquoted attribute value and a mismatched close tag are Errs, because a silent
// partial parse would surface as a machine that is quietly not the one in the file. Element
// text content is dropped.

use std::collections::HashMap;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct XmlNode {
    pub name: String,
    pub attrs: HashMap<String, String>,
    pub children: Vec<XmlNode>,
}

impl XmlNode {
    pub fn attr_or(&self, key: &str, default: &str) -> String {
        match self.attrs.get(key) {
            Some(v) => v.clone(),
            None => default.to_string(),
        }
    }

    fn from_element(el: roxmltree::Node) -> XmlNode {
        XmlNode {
            name: el.tag_name().name().to_string(),
            attrs: el
                .attributes()
                .map(|a| (a.name().to_string(), a.value().to_string()))
                .collect(),
            children: el
                .children()
                .filter(roxmltree::Node::is_element)
                .map(XmlNode::from_element)
                .collect(),
        }
    }
}

/// The document's single root; a document without one is an error rather than an empty tree.
pub fn parse_document(src: &str) -> Result<XmlNode, String> {
    let doc = roxmltree::Document::parse(src).map_err(|err| format!("simu.xml: {err}"))?;
    Ok(XmlNode::from_element(doc.root_element()))
}
