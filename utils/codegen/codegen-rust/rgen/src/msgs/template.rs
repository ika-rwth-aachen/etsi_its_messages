const MSG_TEMPLATE: &str = r#"# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
# Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

{definition}"#;

macro_rules! licensed {
    ($definition:expr) => {
        MSG_TEMPLATE.replace("{definition}", $definition)
    };
}

pub fn typealias_template(comments: &str, name: &str, alias: &str, annotations: &str) -> String {
    licensed!(&format!(
        "## TYPE-ALIAS {name}\n\
        {comments}\n\
        {alias} value\n\
        {annotations}"
    ))
}

pub fn integer_value_template(comments: &str, name: &str, vtype: &str, value: &str) -> String {
    licensed!(&format!(
        "## INTEGER-DEF {name}\n\
        {comments}\n\
        {vtype} {name} = {value}"
    ))
}

pub fn lazy_static_value_template(comments: &str, name: &str, vtype: &str, value: &str) -> String {
    licensed!(&format!(
        "## VALUE {name}\n\
        {comments}\n\
        {vtype} value = {value}"
    ))
}

pub fn integer_template(
    comments: &str,
    name: &str,
    constraints: &str,
    integer_type: &str,
    dvalues: &str,
) -> String {
    let typed_dvalues = dvalues
        .replace("{type}", &integer_type)
        .replace("{prefix}", "");
    licensed!(&format!(
        "## INTEGER {name}\n\
        {comments}\n\
        {integer_type} value\n\
        {constraints}\n\
        {typed_dvalues}\n"
    ))
}

pub fn generalized_time_template(_comments: &str, _name: &str, _annotations: &str) -> String {
    todo!();
}

pub fn utc_time_template(_comments: &str, _name: &str, _annotations: &str) -> String {
    todo!();
}

pub fn bit_string_template(comments: &str, name: &str, constraints: &str, dvalues: &str) -> String {
    let typed_dvalues = dvalues
        .replace("{type}", "uint8")
        .replace("{prefix}", "BIT_INDEX_");
    licensed!(&format!(
        "## BIT-STRING {name}\n\
        {comments}\n\
        uint8[] value\n\
        uint8 bits_unused\n\
        {constraints}\n\
        {typed_dvalues}\n"
    ))
}

pub fn octet_string_template(comments: &str, name: &str, constraints: &str) -> String {
    licensed!(&format!(
        "## OCTET-STRING {name}\n\
        {comments}\n\
        uint8[] value\n\
        {constraints}\n"
    ))
}

pub fn char_string_template(
    comments: &str,
    name: &str,
    string_type: &str,
    constraints: &str,
) -> String {
    licensed!(&format!(
        "## {string_type} {name}\n\
        {comments}\n\
        string value\n\
        {constraints}\n"
    ))
}

pub fn boolean_template(comments: &str, name: &str, constraints: &str) -> String {
    licensed!(&format!(
        "## BOOLEAN {name}\n\
        {comments}\n\
        bool value\n\
        {constraints}\n"
    ))
}

pub fn primitive_value_template(
    _comments: &str,
    _name: &str,
    _type_name: &str,
    _assignment: &str,
) -> String {
    todo!();
}

pub fn enum_value_template(
    _comments: &str,
    _name: &str,
    _enumerated: &str,
    _enumeral: &str,
) -> String {
    todo!();
}

pub fn null_template(_comments: &str, _name: &str, _annotations: &str) -> String {
    todo!();
}

pub fn any_template(_comments: &str, _name: &str, _annotations: &str) -> String {
    todo!();
}

pub fn oid_template(_comments: &str, _name: &str, _annotations: &str) -> String {
    todo!();
}

pub fn enumerated_template(
    comments: &str,
    name: &str,
    enum_members: &str,
    annotations: &str,
) -> String {
    licensed!(&format!(
        "## ENUMERATED {name}\n\
        {comments}\n\
        uint8 value\n\
        {enum_members}\n\
        {annotations}"
    ))
}

pub fn _sequence_or_set_value_template(
    _comments: &str,
    _name: &str,
    _vtype: &str,
    _members: &str,
) -> String {
    todo!()
}

#[allow(clippy::too_many_arguments)]
pub fn sequence_or_set_template(
    comments: &str,
    name: &str,
    members: &str,
    nested_members: Vec<String>,
    annotations: &str,
    default_methods: &str,
    class_fields: &str,
) -> String {
    licensed!(&vec![
        &format!("## SEQUENCE {name}"),
        comments,
        members,
        &nested_members.join("\n"),
        annotations,
        default_methods,
        class_fields
    ]
    .into_iter()
    .filter(|s| !s.is_empty())
    .collect::<Vec<_>>()
    .join("\n"))
}

pub fn sequence_or_set_of_template(
    _is_set_of: bool,
    comments: &str,
    name: &str,
    _anonymous_item: &str,
    member_type: &str,
    constraints: &str,
) -> String {
    licensed!(&format!(
        "## SEQUENCE-OF {name}\n\
        {comments}\n\
        {member_type}[] array\n\
        {constraints}\n"
    ))
}

pub fn choice_value_template(
    _comments: &str,
    _name: &str,
    _type_id: &str,
    _choice_name: &str,
    _inner_decl: &str,
) -> String {
    todo!();
}

pub fn const_choice_value_template(
    _comments: &str,
    _name: &str,
    _type_id: &str,
    _choice_name: &str,
    _inner_decl: &str,
) -> String {
    todo!();
}

pub fn choice_template(
    comments: &str,
    name: &str,
    options: &str,
    nested_options: Vec<String>,
    annotations: &str,
) -> String {
    licensed!(&vec![
        &format!("## CHOICE {name}"),
        comments,
        "uint8 choice\n",
        options,
        &nested_options.join("\n"),
        annotations
    ]
    .into_iter()
    .filter(|s| !s.is_empty())
    .collect::<Vec<_>>()
    .join("\n"))
}
