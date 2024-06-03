use rasn_compiler::prelude::ir::{ASN1Information, ASN1Type};
use rasn_compiler::prelude::*;

mod builder;
mod template;
mod utils;

pub struct Msgs;

use builder::*;

fn generate(tld: ToplevelDefinition) -> Result<String, GeneratorError> {
    match tld {
        ToplevelDefinition::Type(t) => {
            if t.parameterization.is_some() {
                return Ok("".into());
            }
            match t.ty {
                ASN1Type::Null => generate_null(t),
                ASN1Type::Boolean(_) => generate_boolean(t),
                ASN1Type::Integer(_) => generate_integer(t),
                ASN1Type::Enumerated(_) => generate_enumerated(t),
                ASN1Type::BitString(_) => generate_bit_string(t),
                ASN1Type::CharacterString(_) => generate_character_string(t),
                ASN1Type::Sequence(_) | ASN1Type::Set(_) => generate_sequence_or_set(t),
                ASN1Type::SequenceOf(_) | ASN1Type::SetOf(_) => generate_sequence_or_set_of(t),
                ASN1Type::ElsewhereDeclaredType(_) => generate_typealias(t),
                ASN1Type::Choice(_) => generate_choice(t),
                ASN1Type::OctetString(_) => generate_octet_string(t),
                ASN1Type::Time(_) => unimplemented!("rasn does not support TIME types yet!"),
                ASN1Type::Real(_) => Err(GeneratorError {
                    kind: GeneratorErrorType::NotYetInplemented,
                    details: "Real types are currently unsupported!".into(),
                    top_level_declaration: None,
                }),
                ASN1Type::ObjectIdentifier(_) => generate_oid(t),
                ASN1Type::InformationObjectFieldReference(_)
                | ASN1Type::EmbeddedPdv
                | ASN1Type::External => generate_any(t),
                ASN1Type::GeneralizedTime(_) => generate_generalized_time(t),
                ASN1Type::UTCTime(_) => generate_utc_time(t),
                ASN1Type::ChoiceSelectionType(_) => unreachable!(),
            }
        }
        ToplevelDefinition::Value(v) => generate_value(v),
        ToplevelDefinition::Information(i) => match i.value {
            ASN1Information::ObjectSet(_) => generate_information_object_set(i),
            _ => Ok("".into()),
        },
    }
}
