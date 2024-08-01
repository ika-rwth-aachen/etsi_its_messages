use clap::Parser;
use regex::Regex;

use rasn_compiler::prelude::*;
use ros_backend::conversion::{Conversion, ConversionOptions};

#[derive(Parser, Debug)]
struct Cli {
    /// Main PDU name
    #[arg(short, long)]
    pdu: String,
    #[clap(short, long)]
    /// Output directory
    out: std::path::PathBuf,
    /// ASN.1 files to compile
    paths: Vec<std::path::PathBuf>,
}

fn main() {
    let args = Cli::parse();

    let config = ConversionOptions {
        main_pdu: args.pdu,
    };
    let backend = Conversion::from_config(config);

    // Compile conversion headers
    let compiler_res = Compiler::<Conversion, _>::new()
        .with_backend(backend)
        .add_asn_sources_by_path(args.paths.iter())
        .compile_to_string();
    let generated = &compiler_res.unwrap().generated;

    // Split generated code into individual messages
    let re_name = Regex::new(r"\/\/\/\/\s([\w-]+)\s(\w+)\b").unwrap();
    let re_def = Regex::new(r"#<typedef>\n((.|\n)*?)#</typedef>").unwrap();
    generated.split_inclusive("</typedef>").for_each(|s| {
        if let Some(def_caps) = re_def.captures(s) {
            let definition = def_caps.get(1).unwrap().as_str();
            let name = if let Some(name_caps) = re_name.captures(definition) {
                name_caps.get(2).unwrap().as_str()
            } else {
                "unknown"
            };
            let path = args.out.join(format!("convert{}.h", name));
            std::fs::write(path, definition).unwrap();
        }
    });
}
