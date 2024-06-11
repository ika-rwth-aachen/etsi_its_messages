use rasn_compiler::prelude::{ir::ASN1Type, *};

mod builder;
mod template;
mod utils;

#[derive(Default)]
pub struct Conversion {
    options: ConversionOptions,
}
pub struct ConversionOptions {
    main_pdu: String,
}
impl Default for ConversionOptions {
    fn default() -> Self {
        Self {
            main_pdu: "pdu".into(),
        }
    }
}
impl Conversion {
    pub fn set_main_pdu_name(mut self, main_pdu_name: &str) -> Self {
        self.options.main_pdu = main_pdu_name.to_owned();
        self
    }
}

use builder::*;

fn generate(
    options: &ConversionOptions,
    tld: ToplevelDefinition,
) -> Result<String, GeneratorError> {
    match tld {
        ToplevelDefinition::Type(t) => {
            if t.parameterization.is_some() {
                return Ok("".into());
            }
            match t.ty {
                ASN1Type::Null => generate_null(t),
                ASN1Type::Boolean(_) => generate_boolean(&options, t),
                ASN1Type::Integer(_) => generate_integer(&options, t),
                ASN1Type::Enumerated(_) => generate_enumerated(&options, t),
                ASN1Type::BitString(_) => generate_bit_string(&options, t),
                ASN1Type::CharacterString(_) => generate_character_string(&options, t),
                ASN1Type::Sequence(_) | ASN1Type::Set(_) => generate_sequence_or_set(&options, t),
                ASN1Type::SequenceOf(_) | ASN1Type::SetOf(_) => {
                    generate_sequence_or_set_of(&options, t)
                }
                ASN1Type::ElsewhereDeclaredType(_) => generate_typealias(&options, t),
                ASN1Type::Choice(_) => generate_choice(&options, t),
                ASN1Type::OctetString(_) => generate_octet_string(&options, t),
                ASN1Type::Time(_) => unimplemented!("TIME types are currently unsupported!"),
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
            _ => Ok("".into()),
        },
    }
}
