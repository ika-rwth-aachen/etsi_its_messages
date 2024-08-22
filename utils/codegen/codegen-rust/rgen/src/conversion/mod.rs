use std::error::Error;

use rasn_compiler::prelude::*;

mod builder;
mod template;
mod utils;

#[derive(Debug, Default)]
pub struct Conversion {
    options: ConversionOptions,
}

#[derive(Debug)]
pub struct ConversionOptions {
    /// The name of the main PDU.
    pub main_pdu: String,
}

impl Default for ConversionOptions {
    fn default() -> Self {
        Self {
            main_pdu: "pdu".into(),
        }
    }
}

impl Backend for Conversion {
    type Config = ConversionOptions;

    const FILE_EXTENSION: &'static str = "";

    fn from_config(config: Self::Config) -> Self {
        Self { options: config }
    }

    fn config(&self) -> &Self::Config {
        &self.options
    }

    fn generate_module(
        &self,
        tlds: Vec<ToplevelDefinition>,
    ) -> Result<GeneratedModule, GeneratorError> {
        let tlds = self.merge_tlds(tlds);
        let (pdus, warnings): (Vec<String>, Vec<Box<dyn Error>>) =
            tlds.into_iter().fold((vec![], vec![]), |mut acc, tld| {
                match self.generate_tld(tld) {
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

    fn generate(&self, tld: ToplevelDefinition) -> Result<String, GeneratorError> {
        self.generate_tld(tld).map(|ts| ts.to_string())
    }
}

