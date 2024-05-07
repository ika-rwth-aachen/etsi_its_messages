pub fn typealias_template(comments: &str, name: &str, alias: &str, annotations: &str) -> String {
    format!(
        "## TYPE-ALIAS {name}\n\
        {comments}\n\
        {alias} value\n\
        {annotations}"
    )
}

pub fn integer_value_template(comments: &str, name: &str, vtype: &str, value: &str) -> String {
    format!(
        "## INTEGER-DEF {name}\n\
        {comments}\n\
        {vtype} {name} = {value}"
    )
}

pub fn lazy_static_value_template(comments: &str, name: &str, vtype: &str, value: &str) -> String {
    format!(
        "## VALUE {name}\n\
        {comments}\n\
        {vtype} value = {value}"
    )
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
    format!(
        "## INTEGER {name}\n\
        {comments}\n\
        {integer_type} value\n\n\
        {constraints}\n\n\
        {typed_dvalues}"
    )
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
    format!(
        "## BIT-STRING {name}\n\
        {comments}\n\
        uint8[] value\n\
        uint8 bits_unused\n\
        {constraints}\n\n\
        {typed_dvalues}"
    )
}

pub fn octet_string_template(comments: &str, name: &str, annotations: &str) -> String {
    format!(
        "## OCTET-STRING {name}\n\
        {comments}\n\
        uint8[] value\n\
        {annotations}"
    )
}

pub fn char_string_template(
    comments: &str,
    name: &str,
    string_type: &str,
    annotations: &str,
) -> String {
    format!(
        "## {string_type} {name}\n\
        {comments}\n\
        string value\n\
        {annotations}"
    )
}

pub fn boolean_template(comments: &str, name: &str, annotations: &str) -> String {
    format!(
        "## BOOLEAN {name}\n\
        {comments}\n\
        bool value\n\
        {annotations}"
    )
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
    extensible: &str,
    enum_members: &str,
    annotations: &str,
) -> String {
    format!(
        "## ENUMERATED {name} {extensible}\n\
        {comments}\n\
        uint8 value\n\
        {enum_members}\n\
        {annotations}"
    )
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
    extensible: &str,
    members: &str,
    nested_members: Vec<String>,
    annotations: &str,
    default_methods: &str,
    class_fields: &str,
) -> String {
    format!(
        "## SEQUENCE {name} {extensible}\n\
        {comments}\n\
        {members}\n\
        {}\n\
        {annotations}\n\
        {default_methods}\n\
        {class_fields}",
        nested_members.join("\n")
    )
}

pub fn sequence_or_set_of_template(
    _is_set_of: bool,
    comments: &str,
    name: &str,
    _anonymous_item: &str,
    member_type: &str,
    constraints: &str,
) -> String {
    format!(
        "## SEQUENCE-OF {name}\n\
        {comments}\n\
        {member_type}[] array\n\
        {constraints}"
    )
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
    extensible: &str,
    options: &str,
    nested_options: Vec<String>,
    annotations: &str,
) -> String {
    format!(
        "## CHOICE {name} {extensible}\n\
        {comments}\n\
        uint8 choice\n\n\
        {options}\n\
        {}\n\
        {annotations}",
        nested_options.join("\n")
    )
}
