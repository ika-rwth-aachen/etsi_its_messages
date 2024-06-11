use std::error::Error;

use rasn_compiler::prelude::{ir::*, *};

use crate::common::{to_ros_const_case, IntegerTypeExt};
use crate::conversion::{generate, Conversion, ConversionOptions};
use crate::conversion::{template::*, utils::*};

pub(crate) const INNER_ARRAY_LIKE_PREFIX: &str = "Anonymous_";

impl Backend for Conversion {
    fn generate_module(
        &self,
        tlds: Vec<ToplevelDefinition>,
    ) -> Result<GeneratedModule, GeneratorError> {
        let tlds = merge_tlds(tlds);
        let (pdus, warnings): (Vec<String>, Vec<Box<dyn Error>>) =
            tlds.into_iter().fold((vec![], vec![]), |mut acc, tld| {
                match generate(&self.options, tld) {
                    Ok(s) => {
                        s.len().gt(&0).then(|| {
                            acc.0.push(format!(
                                "#<typedef>\n\
                                                {s}\n\
                                                #</typedef>"
                            ))
                        });
                        acc
                    }
                    Err(e) => {
                        acc.1.push(Box::new(e));
                        acc
                    }
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

    // Add value to type's distinguished values
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

    // Resolve object set references
    let mut object_sets = Vec::<(String, ObjectSet)>::new();
    merged_tlds.iter().for_each(|tld| {
        if let ToplevelDefinition::Information(i) = tld {
            if let ASN1Information::ObjectSet(os) = &i.value {
                object_sets.push((tld.name().clone(), os.clone()));
            }
        }
    });
    merged_tlds.iter_mut().for_each(|tld| {
        if let ToplevelDefinition::Type(t) = tld {
            if let ASN1Type::Sequence(ref mut s) = t.ty {
                s.members.iter_mut().for_each(|m| {
                    if let ASN1Type::InformationObjectFieldReference(ref mut r) = m.ty {
                        if let Constraint::TableConstraint(ref mut c) = r.constraints[0] {
                            if let ObjectSetValue::Reference(ref mut osvr) = c.object_set.values[0]
                            {
                                object_sets
                                    .iter()
                                    .find(|s| s.0 == *osvr)
                                    .and_then(|(_, os)| {
                                        let mut os = os.clone();
                                        os.values.push(c.object_set.values[0].clone());
                                        c.object_set = os;
                                        Some(())
                                    });
                            }
                        }
                    }
                });
            }
        }
    });
    merged_tlds
}

pub fn generate_typealias(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::ElsewhereDeclaredType(dec) = &tld.ty {
        Ok(typealias_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
            &dec.identifier,
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

pub fn generate_integer(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::Integer(_) = tld.ty {
        Ok(integer_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected INTEGER top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_bit_string(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::BitString(_) = tld.ty {
        Ok(bit_string_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected BIT STRING top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_octet_string(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::OctetString(ref _oct_str) = tld.ty {
        Ok(octet_string_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected OCTET STRING top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_character_string(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::CharacterString(ref char_str) = tld.ty {
        Ok(char_string_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
            &string_type(&char_str.ty)?,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected Character String top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_boolean(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::Boolean(_) = tld.ty {
        Ok(boolean_template(
            options,
            &format_comments(&tld.comments)?,
            &tld.name,
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
        ASN1Value::Null if ty == NULL => todo!(),
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
        ASN1Value::LinkedStructLikeValue(s) => {
            let _members = s
                .iter()
                .map(|(_, _, val)| value_to_tokens(val.value(), None))
                .collect::<Result<Vec<String>, _>>()?;
            todo!()
        }
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
    Ok(any_template(&format_comments(&tld.comments)?, &tld.name))
}

pub fn generate_generalized_time(tld: ToplevelTypeDefinition) -> Result<String, GeneratorError> {
    if let ASN1Type::GeneralizedTime(_) = &tld.ty {
        Ok(generalized_time_template(
            &format_comments(&tld.comments)?,
            &tld.name,
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
        Ok(oid_template(&format_comments(&tld.comments)?, &tld.name))
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
        Ok(null_template(&format_comments(&tld.comments)?, &tld.name))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected NULL top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_enumerated(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::Enumerated(_) = tld.ty {
        Ok(enumerated_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected ENUMERATED top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_choice(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    if let ASN1Type::Choice(ref choice) = tld.ty {
        let members = get_choice_members_names(choice);
        Ok(choice_template(
            &options,
            &format_comments(&tld.comments)?,
            &tld.name,
            &members,
        ))
    } else {
        Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected CHOICE top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        ))
    }
}

pub fn generate_sequence_or_set(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
    match tld.ty {
        ASN1Type::Sequence(ref seq) | ASN1Type::Set(ref seq) => {
            let members = get_sequence_or_set_members_names(seq);
            Ok(sequence_or_set_template(
                options,
                &format_comments(&tld.comments)?,
                &tld.name,
                members,
            ))
        }
        _ => Err(GeneratorError::new(
            Some(ToplevelDefinition::Type(tld)),
            "Expected SEQUENCE top-level declaration",
            GeneratorErrorType::Asn1TypeMismatch,
        )),
    }
}

pub fn generate_sequence_or_set_of(
    options: &ConversionOptions,
    tld: ToplevelTypeDefinition,
) -> Result<String, GeneratorError> {
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
        n => Some(generate(
            &ConversionOptions::default(),
            ToplevelDefinition::Type(ToplevelTypeDefinition {
                parameterization: None,
                comments: format!(
                    " Anonymous {} OF member ",
                    if is_set_of { "SET" } else { "SEQUENCE" }
                ),
                name: String::from(INNER_ARRAY_LIKE_PREFIX) + &tld.name,
                ty: n.clone(),
                tag: None,
                index: None,
            }),
        )?),
    }
    .unwrap_or_default();
    let member_type = match seq_or_set_of.element_type.as_ref() {
        ASN1Type::ElsewhereDeclaredType(d) => d.identifier.clone(),
        _ => format!("Anonymous{}", &tld.name),
    };
    Ok(sequence_or_set_of_template(
        &options,
        is_set_of,
        &format_comments(&tld.comments)?,
        &tld.name,
        &anonymous_item,
        &member_type,
    ))
}
