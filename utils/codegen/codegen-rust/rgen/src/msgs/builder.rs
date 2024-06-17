use std::{collections::BTreeMap, error::Error};

use rasn_compiler::prelude::ir::*;
use rasn_compiler::prelude::*;

use crate::common::*;
use crate::msgs::{generate, Msgs};
use crate::msgs::{template::*, utils::*};

pub(crate) const INNER_ARRAY_LIKE_PREFIX: &str = "Anonymous_";

impl Backend for Msgs {
    fn generate_module(
        &self,
        tlds: Vec<ToplevelDefinition>,
    ) -> Result<GeneratedModule, GeneratorError> {
        let tlds = merge_tlds(tlds);
        let (pdus, warnings): (Vec<String>, Vec<Box<dyn Error>>) =
            tlds.into_iter()
                .fold((vec![], vec![]), |mut acc, tld| match generate(tld) {
                    Ok(s) => {
                        s.len().gt(&0).then(|| {
                            acc.0.push(format!(
                                "<typedef>\n\
                                 {s}\n\
                                 </typedef>"
                            ))
                        });
                        acc
                    }
                    Err(e) => {
                        acc.1.push(Box::new(e));
                        acc
                    }
                });
        Ok(GeneratedModule {
            generated: Some(format!("{}", pdus.join("\n\n"))),
            warnings,
        })
    }
}

pub fn merge_tlds(tlds: Vec<ToplevelDefinition>) -> Vec<ToplevelDefinition> {
    let mut merged_tlds = Vec::<ToplevelDefinition>::with_capacity(tlds.len());
    let mut merge_to = Vec::<(&ToplevelDefinition, &String)>::new();
    tlds.iter().for_each(|tld| {
        if let ToplevelDefinition::Value(v) = &tld {
            if let ASN1Value::LinkedIntValue { .. } = &v.value {
                merge_to.push((&tld, &v.associated_type));
            } else {
                merged_tlds.push(tld.clone());
            }
        } else {
            merged_tlds.push(tld.clone());
        }
    });
    merge_to.iter().for_each(|(tld, ty)| {
        for t in &mut merged_tlds {
            if let ToplevelDefinition::Type(tt) = t {
                if tt.name == **ty {
                    // Add value to type's distinguished values
                    if let ASN1Type::Integer(ref mut int) = tt.ty {
                        let value = match &tld {
                            ToplevelDefinition::Value(v) => {
                                if let ASN1Value::LinkedIntValue {
                                    integer_type: _,
                                    value,
                                } = &v.value
                                {
                                    value
                                } else {
                                    unreachable!()
                                }
                            }
                            _ => unreachable!(),
                        };
                        match int.distinguished_values {
                            Some(ref mut dv) => dv.push(DistinguishedValue {
                                name: tld.name().clone(),
                                value: *value,
                            }),
                            None => {
                                int.distinguished_values = Some(vec![DistinguishedValue {
                                    name: tld.name().clone(),
                                    value: *value,
                                }]);
                            }
                        }
                    }
                    break;
                }
            }
        }
    });
    merged_tlds
}

pub fn generate_typealias(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::ElsewhereDeclaredType(dec) = &tld.ty {
        Ok(typealias_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &dec.identifier,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected type alias top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_integer_value(tld: ToplevelValueDefinition) -> Result<String, GeneratorError> {
    if let ASN1Value::LinkedIntValue { integer_type, .. } = tld.value {
        let formatted_value = value_to_tokens(&tld.value, None)?;
        let formatted_name = to_ros_const_case(&tld.name);
        let ty = integer_type.to_str();
        if tld.associated_type == INTEGER {
            Ok(lazy_static_value_template(
                &format_comments(&tld.comments)?,
                &formatted_name,
                "int64",
                &formatted_value,
            ))
        } else if integer_type.is_unbounded() {
            Ok(lazy_static_value_template(
                &format_comments(&tld.comments)?,
                &formatted_name,
                ty,
                &formatted_value,
            ))
        } else {
            Ok(integer_value_template(
                &format_comments(&tld.comments)?,
                &formatted_name,
                ty,
                &formatted_value,
            ))
        }
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Value(tld)),
            "Expected INTEGER value top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_integer(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::Integer(ref int) = tld.ty {
        Ok(integer_template(
            &format_comments(&tld.comments)?,
            &to_ros_title_case(&tld.name),
            &format_constraints(true, &int.constraints)?
                .replace("{prefix}", ""),
            int.int_type().to_str(),
            &format_distinguished_values(&int.distinguished_values),
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected INTEGER top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_bit_string(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::BitString(ref bitstr) = tld.ty {
        Ok(bit_string_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &format_constraints(true, &bitstr.constraints)?
                .replace("{prefix}", "")
                .replace("SIZE =", "SIZE_BITS ="),
            &format_distinguished_values(&bitstr.distinguished_values),
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected BIT STRING top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_octet_string(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::OctetString(ref oct_str) = tld.ty {
        Ok(octet_string_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &format_constraints(false, &oct_str.constraints)?
                .replace("{prefix}", ""),
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected OCTET STRING top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_character_string(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::CharacterString(ref char_str) = tld.ty {
        Ok(char_string_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &string_type(&char_str.ty)?,
            &format_constraints(false, &char_str.constraints)?
                .replace("{prefix}", ""),
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected Character String top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_boolean(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::Boolean(_) = tld.ty {
        Ok(boolean_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected BOOLEAN top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

macro_rules! call_template {
    ($fn:ident, $tld:ident, $($args:expr),*) => {
        Ok($fn(
            &format_comments(&$tld.comments)?,
            (&$tld.name),
            $($args),*
        ))
    };
}

pub fn generate_value(tld: ToplevelValueDefinition) -> Result<String, GeneratorError> {
    let ty = tld.associated_type.as_str();
    match &tld.value {
        ASN1Value::Null if ty == NULL => {
            call_template!(
                primitive_value_template,
                tld,
                "quote!(())".into(),
                "quote!(())".into()
            )
        }
        ASN1Value::Null => todo!(),
        ASN1Value::Boolean(b) if ty == BOOLEAN => {
            call_template!(primitive_value_template, tld, "bool", &b.to_string())
        }
        ASN1Value::Boolean(_) => todo!(),
        ASN1Value::LinkedIntValue { .. } => generate_integer_value(tld),
        ASN1Value::BitString(_) if ty == BIT_STRING => todo!(),
        ASN1Value::OctetString(_) if ty == OCTET_STRING => todo!(),
        ASN1Value::Choice {
            variant_name,
            inner_value,
            ..
        } => {
            if inner_value.is_const_type() {
                call_template!(
                    const_choice_value_template,
                    tld,
                    &tld.associated_type,
                    variant_name,
                    &value_to_tokens(inner_value, None)?
                )
            } else {
                call_template!(
                    choice_value_template,
                    tld,
                    &tld.associated_type,
                    &variant_name,
                    &value_to_tokens(inner_value, None)?
                )
            }
        }
        ASN1Value::EnumeratedValue {
            enumerated,
            enumerable,
        } => call_template!(enum_value_template, tld, enumerated, enumerable),
        ASN1Value::Time(_) if ty == GENERALIZED_TIME => todo!(),
        ASN1Value::Time(_) if ty == UTC_TIME => todo!(),
        ASN1Value::LinkedStructLikeValue(_) => todo!(),
        ASN1Value::LinkedNestedValue { .. } => todo!(),
        ASN1Value::ObjectIdentifier(_) if ty == OBJECT_IDENTIFIER => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == NUMERIC_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == VISIBLE_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == IA5_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == UTF8_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == BMP_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == PRINTABLE_STRING => todo!(),
        ASN1Value::LinkedCharStringValue(_, _) if ty == GENERAL_STRING => todo!(),
        ASN1Value::LinkedArrayLikeValue(_) if ty.contains(SEQUENCE_OF) => todo!(),
        ASN1Value::LinkedArrayLikeValue(_) if ty.contains(SET_OF) => todo!(),
        ASN1Value::BitString(_)
        | ASN1Value::Time(_)
        | ASN1Value::LinkedCharStringValue(_, _)
        | ASN1Value::ObjectIdentifier(_)
        | ASN1Value::LinkedArrayLikeValue(_)
        | ASN1Value::ElsewhereDeclaredValue { .. }
        | ASN1Value::OctetString(_) => todo!(),
        _ => Ok("".to_string()),
    }
}

pub fn generate_any(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    Ok(any_template(
        &format_comments(&tld.comments)?,
        &tld.name,
        "",
    ))
}

pub fn generate_generalized_time(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::GeneralizedTime(_) = &tld.ty {
        Ok(generalized_time_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected GeneralizedTime top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_utc_time(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::UTCTime(_) = &tld.ty {
        Ok(utc_time_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected UTCTime top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_oid(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::ObjectIdentifier(_oid) = &tld.ty {
        Ok(oid_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected OBJECT IDENTIFIER top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_null(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::Null = tld.ty {
        Ok(null_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected NULL top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_enumerated(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::Enumerated(ref enumerated) = tld.ty {
        Ok(enumerated_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &format_enum_members(enumerated),
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected ENUMERATED top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_choice(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::Choice(ref choice) = tld.ty {
        let inner_options = format_nested_choice_options(choice, &tld.name)?;
        Ok(choice_template(
            &format_comments(&tld.comments)?,
            &tld.name,
            &format_choice_options(choice, &tld.name)?,
            inner_options,
            "",
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected CHOICE top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_sequence_or_set(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    match tld.ty {
        ASN1Type::Sequence(ref seq) | ASN1Type::Set(ref seq) => {
            let declaration = format_sequence_or_set_members(seq, &tld.name)?;
            Ok(sequence_or_set_template(
                &format_comments(&tld.comments)?,
                &tld.name,
                &declaration,
                format_nested_sequence_members(seq, &tld.name)?,
                "",
                &format_default_methods(&seq.members, &tld.name)?,
                "",
            ))
        }
        _ => Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected SEQUENCE top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        )),
    }
}

pub fn generate_sequence_or_set_of(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    let (is_set_of, seq_or_set_of) = match &tld.ty {
        ASN1Type::SetOf(se_of) => (true, se_of),
        ASN1Type::SequenceOf(se_of) => (false, se_of),
        _ => {
            return Err(GeneratorError::new(
                Some(ToplevelDefinition::Type(tld)),
                "Expected SEQUENCE OF top-level declaration",
                GeneratorErrorType::Asn1TypeMismatch,
            ))
        }
    };
    let anonymous_item = match seq_or_set_of.element_type.as_ref() {
        ASN1Type::ElsewhereDeclaredType(_) => None,
        n => Some(generate(ToplevelDefinition::Type(
            ToplevelTypeDefinition {
                parameterization: None,
                comments: format!(
                    " Anonymous {} OF member ",
                    if is_set_of { "SET" } else { "SEQUENCE" }
                ),
                name: String::from(INNER_ARRAY_LIKE_PREFIX) + &tld.name,
                ty: n.clone(),
                tag: None,
                index: None,
            },
        ))?),
    }
    .unwrap_or_default();
    let member_type = match seq_or_set_of.element_type.as_ref() {
        ASN1Type::ElsewhereDeclaredType(d) => d.identifier.clone(),
        _ => format!("Anonymous{}", &tld.name),
    };
    let constraints = format_constraints(true, &seq_or_set_of.constraints)?
                .replace("{prefix}", "");
    Ok(sequence_or_set_of_template(
        is_set_of,
        &format_comments(&tld.comments)?,
        &tld.name,
        &anonymous_item,
        &member_type,
        &constraints,
    ))
}

pub fn generate_information_object_set(
    tld: ToplevelInformationDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Information::ObjectSet(o) = &tld.value {
        let class: &InformationObjectClass = match tld.class {
            Some(ClassLink::ByReference(ref c)) => c,
            _ => {
                return Err(GeneratorError::new(
                    None,
                    "Missing class link in Information Object Set",
                    GeneratorErrorType::MissingClassKey,
                ))
            }
        };
        let mut keys_to_types = o
            .values
            .iter()
            .map(|v| match v {
                ObjectSetValue::Reference(r) => Err(GeneratorError::new(
                    None,
                    &format!("Could not resolve reference of Information Object Set {r}"),
                    GeneratorErrorType::MissingClassKey,
                )),
                ObjectSetValue::Inline(InformationObjectFields::CustomSyntax(_)) => {
                    Err(GeneratorError::new(
                        Some(ToplevelDefinition::Information(tld.clone())),
                        "Unexpectedly encountered unresolved custom syntax!",
                        GeneratorErrorType::MissingClassKey,
                    ))
                }
                ObjectSetValue::Inline(InformationObjectFields::DefaultSyntax(s)) => {
                    resolve_standard_syntax(class, s)
                }
            })
            .collect::<Result<Vec<(ASN1Value, Vec<(usize, ASN1Type)>)>, _>>()?;
        let mut choices = BTreeMap::<String, Vec<(ASN1Value, ASN1Type)>>::new();
        for (key, items) in keys_to_types.drain(..) {
            for (index, item) in items {
                let id = class
                    .fields
                    .get(index)
                    .map(|f| f.identifier.identifier())
                    .ok_or_else(|| GeneratorError {
                        top_level_declaration: Some(ToplevelDefinition::Information(tld.clone())),
                        details: "Could not find class field for index.".into(),
                        kind: GeneratorErrorType::SyntaxMismatch,
                    })?;
                match choices.get_mut(id) {
                    Some(entry) => entry.push((key.clone(), item)),
                    None => {
                        choices.insert(id.clone(), vec![(key.clone(), item)]);
                    }
                }
            }
        }

        if choices.is_empty() {
            for InformationObjectClassField { identifier, .. } in &class.fields {
                choices.insert(identifier.identifier().clone(), Vec::new());
            }
        }

        let name = &tld.name;
        let class_unique_id_type = class
            .fields
            .iter()
            .find_map(|f| (f.is_unique).then(|| f.ty.clone()))
            .flatten()
            .ok_or_else(|| GeneratorError {
                top_level_declaration: None,
                details: "Could not determine unique class identifier type.".into(),
                kind: GeneratorErrorType::SyntaxMismatch,
            })?;
        let class_unique_id_type_name = type_to_tokens(&class_unique_id_type)?;

        let mut field_enums = vec![];
        for (_field_name, fields) in choices.iter() {
            let field_enum_name = name.clone();
            let mut ids = vec![];
            for (index, (id, ty)) in fields.iter().enumerate() {
                let identifier_value = match id {
                    ASN1Value::LinkedElsewhereDefinedValue {
                        can_be_const: false,
                        ..
                    } => {
                        let _tokenized_value =
                            value_to_tokens(id, Some(&class_unique_id_type_name))?;
                        todo!()
                    }
                    ASN1Value::LinkedNestedValue { value, .. }
                        if matches![
                            &**value,
                            ASN1Value::LinkedElsewhereDefinedValue {
                                can_be_const: false,
                                ..
                            }
                        ] =>
                    {
                        let _tokenized_value =
                            value_to_tokens(value, Some(&class_unique_id_type_name))?;
                        todo!()
                    }
                    ASN1Value::LinkedNestedValue { value, .. }
                        if matches![&**value, ASN1Value::LinkedElsewhereDefinedValue { .. }] =>
                    {
                        value_to_tokens(value, Some(&class_unique_id_type_name))?
                    }
                    _ => value_to_tokens(id, Some(&class_unique_id_type_name))?,
                };
                let type_id = type_to_tokens(ty).unwrap_or("type?".into());
                let variant_name = match id {
                    ASN1Value::LinkedElsewhereDefinedValue {
                        identifier: ref_id, ..
                    }
                    | ASN1Value::ElsewhereDeclaredValue {
                        identifier: ref_id, ..
                    } => ref_id.clone(),
                    _ => format!("{field_enum_name}_{index}"),
                };
                if ty.constraints().map_or(true, |c| c.is_empty()) {
                    ids.push((variant_name, type_id, identifier_value));
                }
            }

            let variants = ids.iter().map(|(variant_name, type_id, _)| {
                format!("{type_id} {}", to_ros_snake_case(variant_name))
            });

            field_enums.push(format!(
                "## OPEN-TYPE {field_enum_name}\n{class_unique_id_type_name} choice\n{}",
                variants.fold("".to_string(), |mut acc, v| {
                    acc.push_str(&v);
                    acc.push_str("\n");
                    acc
                }),
            ));
        }

        Ok(field_enums.join("\n"))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Information(tld)),
            "Expected Object Set top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}
