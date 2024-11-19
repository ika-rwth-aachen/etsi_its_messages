use rasn_compiler::intermediate::{
    constraints::Constraint,
    information_object::InformationObjectField,
    types::{Choice, SequenceOrSet},
    ASN1Type, ASN1Value, CharacterStringType, IntegerType,
};
use rasn_compiler::prelude::{ir::*, *};

use crate::common::to_ros_title_case;
use super::Conversion;

macro_rules! error {
    ($kind:ident, $($arg:tt)*) => {
        GeneratorError {
            details: format!($($arg)*),
            top_level_declaration: None,
            kind: GeneratorErrorType::$kind,
        }
    };
}

pub(crate) use error;

#[derive(Clone, Debug)]
pub struct NameType {
    pub name: String,
    pub ty: String,
    pub is_primitive: bool,
    pub inner_types: Option<InnerTypes>,
}

#[derive(Clone, Debug)]
pub enum InnerTypes {
    Choice(InnerTypesChoice),
}

#[derive(Clone, Debug)]
pub struct InnerTypesChoice {
    pub linked_with: String,
    pub options: Vec<NameType>,
}

#[derive(Clone, Debug)]
pub struct NamedSeqMember {
    pub name_type: NameType,
    pub is_optional: bool,
    pub has_default: bool,
}

impl Conversion {
    pub fn format_comments(&self, comments: &str) -> Result<String, GeneratorError> {
        if comments.is_empty() {
            Ok("".into())
        } else {
            let joined = String::from("// ") + &comments.replace('\n', "\n//") + "\n";
            Ok(joined)
        }
    }

    pub fn inner_name(&self, name: &String, parent_name: &String) -> String {
        format!("{}{}", parent_name, name)
    }


    fn get_inner_types_names(&self, ty: &ASN1Type) -> Option<InnerTypes> {
        match ty {
            ASN1Type::InformationObjectFieldReference(r) => {
                if let Constraint::TableConstraint(ref tc) = r.constraints[0] {
                    let object_set = &tc.object_set;
                    let mut names = vec![];
                    for value in &object_set.values {
                        if let ObjectSetValue::Inline(ref i) = value {
                            match i {
                                InformationObjectFields::DefaultSyntax(ds) => {
                                    let mut name = "".to_string();
                                    let mut ty: ASN1Type = ASN1Type::Null;
                                    ds.iter().for_each(|f| match f {
                                        InformationObjectField::TypeField(f) => ty = f.ty.clone(),
                                        InformationObjectField::FixedValueField(f) => {
                                            name = self.value_to_tokens(&f.value, None).unwrap()
                                        }
                                        _ => todo!(),
                                    });
                                    names.push(NameType {
                                        name: name.clone(),
                                        ty: self.constraints_and_type_name(&ty, &name, &"".to_string())
                                            .unwrap()
                                            .1,
                                            is_primitive: ty.is_builtin_type(),
                                            inner_types: None,
                                    });
                                }
                                _ => todo!(),
                            }
                        }
                    }
                    let linked_with = tc
                        .linked_fields
                        .get(0)
                        .map(|f| f.field_name.clone())
                        .unwrap_or_default();
                    Some(InnerTypes::Choice(InnerTypesChoice {
                        linked_with,
                        options: names,
                    }))
                } else {
                    unreachable!()
                }
            }
            _ => None,
        }
    }

    pub fn get_sequence_or_set_members_names(&self, sequence_or_set: &SequenceOrSet) -> Vec<NamedSeqMember> {
        sequence_or_set
            .members
            .iter()
            .map(|member| NamedSeqMember {
                name_type: NameType {
                    name: member.name.clone(),
                    ty: self.constraints_and_type_name(&member.ty, &member.name, &"".to_string())
                        .unwrap()
                        .1,
                        is_primitive: member.ty.is_builtin_type(),
                        inner_types: self.get_inner_types_names(&member.ty),
                },
                is_optional: member.is_optional && (member.default_value.is_none() || self.value_to_tokens(member.default_value.as_ref().unwrap(), None).unwrap() != "0"),
                has_default: member.default_value.is_some(),
            })
        .collect::<Vec<NamedSeqMember>>()
    }

    pub fn get_choice_members_names(&self, choice: &Choice) -> Vec<NameType> {
        choice
            .options
            .iter()
            .map(|member| 
                NameType {
                    name: member.name.clone(),
                    ty: self.constraints_and_type_name(&member.ty, &member.name, &"".to_string())
                        .unwrap()
                        .1,
                        is_primitive: member.ty.is_builtin_type(),
                        inner_types: None,
                }
            ).collect::<Vec<NameType>>()
    }

    fn constraints_and_type_name(
        &self,
        ty: &ASN1Type,
        name: &String,
        parent_name: &String,
    ) -> Result<(Vec<Constraint>, String), GeneratorError> {
        Ok(match ty {
            ASN1Type::Null => (vec![], "byte".into()),
            ASN1Type::Boolean(b) => (b.constraints.clone(), "BOOLEAN".into()),
            ASN1Type::Integer(i) => (i.constraints.clone(), "INTEGER".into()),
            ASN1Type::Real(_) => (vec![], "float64".into()),
            ASN1Type::ObjectIdentifier(_o) => todo!(),
            ASN1Type::BitString(b) => (b.constraints.clone(), "uint8[]".into()),
            ASN1Type::OctetString(o) => (o.constraints.clone(), "uint8[]".into()),
            ASN1Type::GeneralizedTime(_o) => todo!(),
            ASN1Type::UTCTime(_o) => todo!(),
            ASN1Type::Time(_t) => todo!(),
            ASN1Type::CharacterString(c) => (
                c.constraints.clone(),
                self.string_type(&c.ty).unwrap_or("STRING".into()),
            ),
            ASN1Type::Enumerated(_)
                | ASN1Type::Choice(_)
                | ASN1Type::Sequence(_)
                | ASN1Type::SetOf(_)
                | ASN1Type::Set(_) => (vec![], self.inner_name(name, parent_name)),
            ASN1Type::SequenceOf(s) => {
                let (_, inner_type) = self.constraints_and_type_name(&s.element_type, name, parent_name)?;
                (s.constraints().clone(), format!("{inner_type}[]").into())
            }
            ASN1Type::ElsewhereDeclaredType(e) => {
                (e.constraints.clone(), to_ros_title_case(&e.identifier))
            }
            ASN1Type::InformationObjectFieldReference(_)
                | ASN1Type::EmbeddedPdv
                | ASN1Type::External => {
                    let tx = &ty.constraints().unwrap()[0];
                    let rname = if let Constraint::TableConstraint(ref tc) = tx {
                        tc.object_set
                            .values
                            .iter()
                            .find_map(|v| match v {
                                ObjectSetValue::Reference(ref r) => Some(r.clone()),
                                _ => None,
                            })
                        .unwrap_or_default()
                    } else {
                        "".to_string()
                    };
                    (vec![], rname)
                }
            ASN1Type::ChoiceSelectionType(_) => unreachable!(),
        })
    }

    pub fn string_type(&self, c_type: &CharacterStringType) -> Result<String, GeneratorError> {
        match c_type {
            CharacterStringType::NumericString => Ok("NumericString".into()),
            CharacterStringType::VisibleString => Ok("VisibleString".into()),
            CharacterStringType::IA5String => Ok("IA5String".into()),
            CharacterStringType::TeletexString => Ok("TeletexString".into()),
            CharacterStringType::VideotexString => Ok("VideotexString".into()),
            CharacterStringType::GraphicString => Ok("GraphicString".into()),
            CharacterStringType::GeneralString => Ok("GeneralString".into()),
            CharacterStringType::UniversalString => Ok("UniversalString".into()),
            CharacterStringType::UTF8String => Ok("UTF8String".into()),
            CharacterStringType::BMPString => Ok("BMPString".into()),
            CharacterStringType::PrintableString => Ok("PrintableString".into()),
        }
    }

    pub fn value_to_tokens(
        &self,
        value: &ASN1Value,
        type_name: Option<&String>,
    ) -> Result<String, GeneratorError> {
        match value {
            ASN1Value::All => Err(error!(
                    NotYetInplemented,
                    "All values are currently unsupported!"
            )),
            ASN1Value::Null => todo!(),
            ASN1Value::Choice { inner_value, .. } => {
                if let Some(_ty_n) = type_name {
                    todo!()
                } else {
                    Err(error!(
                            Unidentified,
                            "A type name is needed to stringify choice value {:?}", inner_value
                    ))
                }
            }
            ASN1Value::OctetString(o) => {
                let _bytes = o.iter().map(|byte| *byte);
                todo!()
            }
            ASN1Value::SequenceOrSet(_) => Err(error!(
                    Unidentified,
                    "Unexpectedly encountered unlinked struct-like ASN1 value!"
            )),
            ASN1Value::LinkedStructLikeValue(fields) => {
                if let Some(_ty_n) = type_name {
                    let _tokenized_fields = fields
                        .iter()
                        .map(|(_, _, val)| self.value_to_tokens(val.value(), None))
                        .collect::<Result<Vec<String>, _>>()?;
                    todo!()
                } else {
                    Err(error!(
                            Unidentified,
                            "A type name is needed to stringify sequence value {:?}", value
                    ))
                }
            }
            ASN1Value::Boolean(b) => Ok(b.to_string()),
            ASN1Value::Integer(i) => Ok(i.to_string()),
            ASN1Value::String(s) => Ok(s.to_string()),
            ASN1Value::Real(r) => Ok(r.to_string()),
            ASN1Value::BitString(b) => {
                let _bits = b.iter().map(|bit| bit.to_string());
                todo!()
            }
            ASN1Value::EnumeratedValue {
                enumerated,
                enumerable,
            } => Ok(format!("{}_{}", enumerated, enumerable)),
            ASN1Value::LinkedElsewhereDefinedValue { identifier: e, .. }
            | ASN1Value::ElsewhereDeclaredValue { identifier: e, .. } => Ok(e.to_string()),
            ASN1Value::ObjectIdentifier(oid) => {
                let _arcs = oid
                    .0
                    .iter()
                    .filter_map(|arc| arc.number.map(|id| id.to_string()));
                todo!()
            }
            ASN1Value::Time(_t) => match type_name {
                Some(_time_type) => todo!(),
                None => todo!(),
            },
            ASN1Value::LinkedArrayLikeValue(seq) => {
                let _elems = seq
                    .iter()
                    .map(|v| self.value_to_tokens(v, None))
                    .collect::<Result<Vec<_>, _>>()?;
                todo!()
            }
            ASN1Value::LinkedNestedValue {
                supertypes: _,
                value,
            } => Ok(self.value_to_tokens(value, type_name)?),
            ASN1Value::LinkedIntValue {
                integer_type,
                value,
            } => {
                let val = *value;
                match integer_type {
                    IntegerType::Unbounded => Ok(val.to_string()),
                    _ => Ok(val.to_string()),
                }
            }
            ASN1Value::LinkedCharStringValue(string_type, value) => {
                let _val = value;
                match string_type {
                    CharacterStringType::NumericString => {
                        todo!()
                    }
                    CharacterStringType::VisibleString => {
                        todo!()
                    }
                    CharacterStringType::IA5String => {
                        todo!()
                    }
                    CharacterStringType::UTF8String => todo!(),
                    CharacterStringType::BMPString => {
                        todo!()
                    }
                    CharacterStringType::PrintableString => {
                        todo!()
                    }
                    CharacterStringType::GeneralString => {
                        todo!()
                    }
                    CharacterStringType::VideotexString
                        | CharacterStringType::GraphicString
                        | CharacterStringType::UniversalString
                        | CharacterStringType::TeletexString => Err(GeneratorError::new(
                                None,
                                &format!("{:?} values are currently unsupported!", string_type),
                                GeneratorErrorType::NotYetInplemented,
                        )),
                }
            }
        }
    }
}
