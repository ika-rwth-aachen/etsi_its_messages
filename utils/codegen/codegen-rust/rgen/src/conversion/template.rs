use crate::common::{to_ros_const_case, to_ros_snake_case, to_ros_title_case, to_c_title_case};
use crate::conversion::utils::{InnerTypes, NameType, NamedSeqMember};
use crate::conversion::ConversionOptions;

use std::collections::{HashMap, HashSet};

const CONVERSION_TEMPLATE: &str = 
r#"/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

//// {asn1_type} {name}

#pragma once
{extra-includes}
#include <etsi_its_{pdu}_coding/{pdu}_{c_filename}.h>
{c_includes}
#ifdef ROS1
{ros1_includes}
namespace {pdu}_msgs = etsi_its_{pdu}_msgs;
#else
{ros2_includes}
namespace {pdu}_msgs = etsi_its_{pdu}_msgs::msg;
#endif


namespace etsi_its_{pdu}_conversion {

void toRos_{type}(const {pdu}_{c_type}_t& in, {pdu}_msgs::{ros_type}& out) {
  {to_ros_members}
}

void toStruct_{type}(const {pdu}_msgs::{ros_type}& in, {pdu}_{c_type}_t& out) {
  memset(&out, 0, sizeof({pdu}_{c_type}_t));

  {to_c_members}
}

}"#;

pub fn conversion_template(
    _comments: &str,
    pdu: &str,
    includes: &Vec<NameType>,
    extra_includes: &Vec<&str>,
    name: &str,
    asn1_type: &str,
    to_ros_members: &str,
    to_c_members: &str,
) -> String {
    let mut c_includes_lines = includes
        .iter()
        .map(|member| {
            if !member.is_primitive {
                format!(
                    "#include <etsi_its_{pdu}_conversion/convert{dep}.h>",
                    pdu = pdu,
                    dep = member.ty
                )
            } else {
                format!(
                    "#include <etsi_its_{pdu}_coding/{dep}.h>",
                    pdu = pdu,
                    dep = member.ty
                ) + "\n"
                    + &format!(
                        "#include <etsi_its_primitives_conversion/convert{dep}.h>",
                        dep = member.ty
                    )
            }
        })
        .collect::<HashSet<String>>()
        .into_iter()
        .collect::<Vec<String>>();
    c_includes_lines.sort();
    let c_includes = c_includes_lines.join("\n");
    let ros1_includes = format!(
        "#include <etsi_its_{pdu}_msgs/{ros_fn}.h>",
        pdu = pdu,
        ros_fn = to_ros_title_case(name)
    );
    let ros2_includes = format!(
        "#include <etsi_its_{pdu}_msgs/msg/{ros_fn}.hpp>",
        pdu = pdu,
        ros_fn = to_ros_snake_case(name)
    );

    let extra_includes = extra_includes
        .iter()
        .fold(
            String::from(if extra_includes.len() > 0 { "\n" } else { "" }), 
            |acc, inc| acc + &format!("#include <{inc}>\n")
        );

    CONVERSION_TEMPLATE
        .replace("{extra-includes}", &extra_includes)
        .replace("{c_includes}", &c_includes)
        .replace("{ros1_includes}", &ros1_includes)
        .replace("{ros2_includes}", &ros2_includes)
        .replace("{asn1_type}", asn1_type)
        .replace("{name}", &to_ros_title_case(name))
        .replace("{c_type}", &to_c_title_case(name))
        .replace("{c_filename}", name)
        .replace("{type}", &to_ros_title_case(name))
        .replace("{ros_type}", &to_ros_title_case(name))
        .replace("{pdu}", pdu)
        .replace("{to_ros_members}", &to_ros_members)
        .replace("{to_c_members}", &to_c_members)
}

pub fn typealias_template(
    options: &ConversionOptions,
    comments: &str,
    name: &str,
    alias: &str,
) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: alias.to_string(),
            is_primitive: false,
            inner_types: None,
        }],
        &vec![],
        name,
        "TYPEALIAS",
        &format!("toRos_{alias}(in, out.value);"),
        &format!("toStruct_{alias}(in.value, out);"),
    )
}

pub fn integer_value_template(_comments: &str, _name: &str, _vtype: &str, _value: &str) -> String {
    todo!()
}

pub fn lazy_static_value_template(
    _comments: &str,
    _name: &str,
    _vtype: &str,
    _value: &str,
) -> String {
    todo!()
}

pub fn integer_template(options: &ConversionOptions, comments: &str, name: &str) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: "INTEGER".to_string(),
            is_primitive: true,
            inner_types: None,
        }],
        &vec![],
        name,
        "INTEGER",
        "etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);",
        "etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);",
    )
}

pub fn generalized_time_template(_comments: &str, _name: &str) -> String {
    todo!();
}

pub fn utc_time_template(_comments: &str, _name: &str) -> String {
    todo!();
}

pub fn bit_string_template(options: &ConversionOptions, comments: &str, name: &str) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: "BIT_STRING".to_string(),
            is_primitive: true,
            inner_types: None,
        }],
        &vec![],
        name,
        "BIT-STRING",
        "etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);\n  \
         out.bits_unused = in.bits_unused;",
        "etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);\n  \
         out.bits_unused = in.bits_unused;",
    )
}

pub fn octet_string_template(options: &ConversionOptions, comments: &str, name: &str) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: "OCTET_STRING".to_string(),
            is_primitive: true,
            inner_types: None,
        }],
        &vec![],
        name,
        "OCTET-STRING",
        "etsi_its_primitives_conversion::toRos_OCTET_STRING(in, out.value);",
        "etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.value, out);",
    )
}

pub fn char_string_template(
    options: &ConversionOptions,
    comments: &str,
    name: &str,
    string_type: &str,
) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: string_type.to_string(),
            is_primitive: true,
            inner_types: None,
        }],
        &vec![],
        name,
        string_type,
        &format!("etsi_its_primitives_conversion::toRos_{string_type}(in, out.value);"),
        &format!("etsi_its_primitives_conversion::toStruct_{string_type}(in.value, out);"),
    )
}

pub fn boolean_template(options: &ConversionOptions, comments: &str, name: &str) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![NameType {
            name: name.to_string(),
            ty: "BOOLEAN".to_string(),
            is_primitive: true,
            inner_types: None,
        }],
        &vec![],
        name,
        "BOOLEAN",
        "etsi_its_primitives_conversion::toRos_BOOLEAN(in, out.value);",
        "etsi_its_primitives_conversion::toStruct_BOOLEAN(in.value, out);",
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

pub fn null_template(_comments: &str, _name: &str) -> String {
    todo!();
}

pub fn any_template(_comments: &str, _name: &str) -> String {
    todo!();
}

pub fn oid_template(_comments: &str, _name: &str) -> String {
    todo!();
}

pub fn enumerated_template(options: &ConversionOptions, comments: &str, name: &str) -> String {
    conversion_template(
        comments,
        &options.main_pdu,
        &vec![],
        &vec![],
        name,
        "ENUMERATED",
        "out.value = in;",
        "out = in.value;",
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
    options: &ConversionOptions,
    comments: &str,
    name: &str,
    members: Vec<NamedSeqMember>,
) -> String {
    let links: HashMap<String, &NamedSeqMember> = members
        .iter()
        .filter_map(|m| {
            if let Some(inner) = &m.name_type.inner_types {
                match inner {
                    InnerTypes::Choice(c) => Some((
                        c.linked_with.clone(),
                        members
                            .iter()
                            .find(|m| m.name_type.name == c.linked_with)
                            .unwrap(),
                    )),
                }
            } else {
                None
            }
        })
        .collect();

    // C -> ROS
    let to_ros_inner_members = |member: &NamedSeqMember| -> String {
        if let Some(inner) = &member.name_type.inner_types {
            let cases = match inner {
                InnerTypes::Choice(c) => {
                    c.options.iter().map(|im| {
                        format!("  case {pdu}_{name}__{c_field_name}_PR_{ty}:\n    \
                                 toRos_{ty}(in.{c_field_name}.choice.{c_member}, out.{r_field_name}.{r_member});\n    \
                                 out.{r_field_name}.choice.value = {pdu}_msgs::{linked_with}::{r_const_member};\n    \
                                 break;", 
                            pdu = &options.main_pdu,
                            linked_with = links.get(&c.linked_with).unwrap().name_type.ty,
                            c_field_name = member.name_type.name,
                            r_field_name = &to_ros_snake_case(&member.name_type.name),
                            ty = im.ty,
                            c_member = to_ros_title_case(&im.name),
                            r_member = to_ros_snake_case(&im.name),
                            r_const_member = to_ros_const_case(&im.name))
                    }).collect::<Vec<String>>().join("\n")
                },
            };
            format!(
                "switch (in.{field_name}.present) {{\n\
                    {cases}\n  \
                    }}",
                field_name = member.name_type.name,
                cases = cases
            )
        } else {
            "".to_string()
        }
    };

    let to_ros_conversion_call = |member: &NamedSeqMember| -> String {
        if !member.name_type.is_primitive {
            if member.name_type.inner_types.is_some() {
                // TODO optional type with inner_types
                to_ros_inner_members(&member)
            } else {
                format!(
                    "toRos_{ty}({deref}in.{c_member}, out.{r_member});",
                    ty = member.name_type.ty,
                    deref = if member.is_optional { "*" } else { "" },
                    c_member = member.name_type.name,
                    r_member = to_ros_snake_case(&member.name_type.name)
                )
            }
        } else {
            format!(
                "etsi_its_primitives_conversion::toRos_{ty}({deref}in.{c_member}, out.{r_member});",
                ty = member.name_type.ty,
                deref = if member.is_optional { "*" } else { "" },
                c_member = member.name_type.name,
                r_member = to_ros_snake_case(&member.name_type.name)
            )
        }
    };
    let to_ros_fmt_member = |member: &NamedSeqMember| -> String {
        if member.is_optional {
            format!(
                "if (in.{c_member}) {{\n    \
                     {conversion}\n  \
                     {present}\
                     }}",
                c_member = member.name_type.name,
                conversion = to_ros_conversion_call(&member),
                present = if !member.has_default {
                    format!(
                        "  out.{r_member}_is_present = true;\n  ",
                        r_member = to_ros_snake_case(&member.name_type.name)
                    )
                } else {
                    "".to_string()
                }
            )
        } else {
            to_ros_conversion_call(&member)
        }
    };
    let to_ros_members = members
        .iter()
        .map(|member| to_ros_fmt_member(&member))
        .collect::<Vec<String>>()
        .join("\n  ");

    // ROS -> C
    let to_c_inner_members = |member: &NamedSeqMember| -> String {
        if let Some(inner) = &member.name_type.inner_types {
            let cases = match inner {
                InnerTypes::Choice(c) => {
                    c.options.iter().map(|im| {
                        format!("  case {pdu}_msgs::{linked_with}::{r_const_member}:\n    \
                                 toStruct_{ty}(in.{r_field_name}.{r_member}, out.{c_field_name}.choice.{c_member});\n    \
                                 out.{c_field_name}.present = {pdu}_{name}__{c_field_name}_PR::{pdu}_{name}__{c_field_name}_PR_{c_member};\n    \
                                 break;", 
                            pdu = &options.main_pdu,
                            linked_with = links.get(&c.linked_with).unwrap().name_type.ty,
                            c_field_name = member.name_type.name,
                            r_field_name = &to_ros_snake_case(&member.name_type.name),
                            ty = im.ty,
                            c_member = to_ros_title_case(&im.name),
                            r_member = to_ros_snake_case(&im.name),
                            r_const_member = to_ros_const_case(&im.name))
                    }).collect::<Vec<String>>().join("\n")
                },
            };
            format!(
                "switch (in.{field_name}.choice.value) {{\n\
                    {cases}\n  \
                    }}",
                field_name = to_ros_snake_case(&member.name_type.name),
                cases = cases
            )
        } else {
            "".to_string()
        }
    };

    let to_c_conversion_call = |member: &NamedSeqMember| -> String {
        if !member.name_type.is_primitive {
            if member.name_type.inner_types.is_some() {
                // TODO optional type with inner_types
                to_c_inner_members(&member)
            } else {
                format!(
                    "toStruct_{ty}(in.{r_member}, {deref}out.{c_member});",
                    ty = member.name_type.ty,
                    deref = if member.is_optional { "*" } else { "" },
                    c_member = member.name_type.name,
                    r_member = to_ros_snake_case(&member.name_type.name)
                )
            }
        } else {
            format!("etsi_its_primitives_conversion::toStruct_{ty}(in.{r_member}, {deref}out.{c_member});", 
                ty = member.name_type.ty,
                deref = if member.is_optional { "*" } else { "" },
                c_member = member.name_type.name,
                r_member = to_ros_snake_case(&member.name_type.name)
            )
        }
    };
    let to_c_fmt_member = |member: &NamedSeqMember| -> String {
        if member.is_optional {
            if !member.has_default {
                if !member.name_type.is_primitive {
                format!(
                    "if (in.{r_member}_is_present) {{\n    \
                         out.{c_member} = ({pdu}_{ty}_t*) calloc(1, sizeof({pdu}_{ty}_t));\n    \
                         {conversion}\n  \
                         }}",
                    ty = member.name_type.ty,
                    c_member = member.name_type.name,
                    conversion = to_c_conversion_call(&member),
                    r_member = to_ros_snake_case(&member.name_type.name),
                    pdu = &options.main_pdu
                )
            } else {
                    format!(
                        "if (in.{r_member}_is_present) {{\n    \
                            out.{c_member} = ({ty}_t*) calloc(1, sizeof({ty}_t));\n    \
                            {conversion}\n  \
                            }}",
                        ty = member.name_type.ty,
                        c_member = member.name_type.name,
                        conversion = to_c_conversion_call(&member),
                        r_member = to_ros_snake_case(&member.name_type.name),
                    )
                }
            } else {
                if !member.name_type.is_primitive {
                format!(
                    "out.{c_member} = ({pdu}_{ty}_t*) calloc(1, sizeof({pdu}_{ty}_t));\n  \
                     {conversion}",
                    ty = member.name_type.ty,
                    c_member = member.name_type.name,
                    conversion = to_c_conversion_call(&member),
                    pdu = &options.main_pdu
                )
                } else {
                    format!(
                        "out.{c_member} = ({ty}_t*) calloc(1, sizeof({ty}_t));\n  \
                         {conversion}",
                        ty = member.name_type.ty,
                        c_member = member.name_type.name,
                        conversion = to_c_conversion_call(&member),
                    )
                }
            }
        } else {
            to_c_conversion_call(&member)
        }
    };
    let to_c_members = members
        .iter()
        .map(|member| to_c_fmt_member(&member))
        .collect::<Vec<String>>()
        .join("\n  ");

    let includes = &members
        .iter()
        .flat_map(|m| {
            if let Some(inners) = &m.name_type.inner_types {
                match inners {
                    InnerTypes::Choice(c) => c.options.clone(),
                }
            } else {
                vec![m.name_type.clone()]
            }
        })
        .collect();

    conversion_template(
        comments,
        &options.main_pdu,
        &includes,
        &vec![],
        name,
        "SEQUENCE",
        &to_ros_members,
        &to_c_members,
    )
}

pub fn sequence_or_set_of_template(
    options: &ConversionOptions,
    _is_set_of: bool,
    comments: &str,
    name: &str,
    _anonymous_item: &str,
    member_type: &str,
) -> String {
    let to_ros_loop = format!(
        "for (int i = 0; i < in.list.count; ++i) {{\n    \
         {pdu}_msgs::{ty} el;\n    \
         toRos_{ty}(*(in.list.array[i]), el);\n    \
         out.array.push_back(el);\n  \
         }}",
        pdu = &options.main_pdu,
        ty = member_type
    );

    let to_c_loop =
        format!("for (int i = 0; i < in.array.size(); ++i) {{\n    \
                 {pdu}_{ty}_t* el = ({pdu}_{ty}_t*) calloc(1, sizeof({pdu}_{ty}_t));\n    \
                 toStruct_{ty}(in.array[i], *el);\n    \
                 if (asn_sequence_add(&out, el)) throw std::invalid_argument(\"Failed to add to A_SEQUENCE_OF\");\n  \
                 }}", 
                ty = member_type,
                pdu = &options.main_pdu);

    conversion_template(
        comments,
        &options.main_pdu,
        &vec![
            NameType {
                name: name.to_string(),
                ty: name.to_string(),
                is_primitive: false,
                inner_types: None,
            },
            NameType {
                name: member_type.to_string(),
                ty: member_type.to_string(),
                is_primitive: false,
                inner_types: None,
            },
        ],
        &vec!["stdexcept"],
        name,
        "SEQUENCE-OF",
        &to_ros_loop,
        &to_c_loop,
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
    options: &ConversionOptions,
    comments: &str,
    name: &str,
    members: &Vec<NameType>,
) -> String {
    let to_ros_members = format!("switch (in.present) {{\n")
        + &members
            .iter()
            .map(|member| {
                if !member.is_primitive {
                    format!(
                        "  case {pdu}_{parent}_PR_{c_member}:\n    \
                         toRos_{ty}(in.choice.{c_member}, out.{r_member});\n    \
                         out.choice = {pdu}_msgs::{parent}::CHOICE_{r_ch_member};",
                        parent = &name,
                        ty = member.ty,
                        pdu = &options.main_pdu,
                        c_member = member.name.replace('-', "_"),
                        r_member = to_ros_snake_case(&member.name),
                        r_ch_member = to_ros_const_case(&member.name)
                    )
                } else {
                    format!(
                        "etsi_its_primitives_conversion::toRos_{ty}(in, out.value);",
                        ty = member.ty,
                    )
                }
            })
            .collect::<Vec<String>>()
            .join("\n    break;\n")
        + "\n    break;\n  default: break;\n  }";

    let to_c_members = format!("switch (in.choice) {{\n")
        + &members
            .iter()
            .map(|member| {
                if !member.is_primitive {
                    format!(
                        "  case {pdu}_msgs::{parent}::CHOICE_{r_ch_member}:\n    \
                         toStruct_{ty}(in.{r_member}, out.choice.{c_member});\n    \
                         out.present = {pdu}_{parent}_PR::{pdu}_{parent}_PR_{c_member};",
                        parent = &name,
                        ty = member.ty,
                        pdu = &options.main_pdu,
                        c_member = member.name.replace('-', "_"),
                        r_member = to_ros_snake_case(&member.name),
                        r_ch_member = to_ros_const_case(&member.name)
                    )
                } else {
                    format!(
                        "etsi_its_primitives_conversion::toStruct_{ty}(in, out.value);",
                        ty = member.ty,
                    )
                }
            })
            .collect::<Vec<String>>()
            .join("\n    break;\n")
        + "\n    break;\n  default: break;\n  }";

    conversion_template(
        comments,
        &options.main_pdu,
        &members,
        &vec![],
        name,
        "CHOICE",
        &to_ros_members,
        &to_c_members,
    )
}
