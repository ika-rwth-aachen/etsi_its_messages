use clap::Parser;
use regex::Regex;

use rasn_compiler::prelude::*;
use ros_backend::msgs::Msgs;

#[derive(Parser, Debug)]
struct Cli {
    #[clap(short, long)]
    /// Output directory
    out: std::path::PathBuf,
    /// ASN.1 files to compile
    paths: Vec<std::path::PathBuf>,
}

fn main() {
    let args = Cli::parse();

    // Compile ROS messages
    let compiler_res = Compiler::<Msgs, _>::new()
        .add_asn_sources_by_path(args.paths.iter())
        .compile_to_string();
    let generated = &compiler_res.unwrap().generated;

    // Split generated code into individual messages
    let re_name = Regex::new(r"<typename>\s*([\w-]+)\s*([\w-]+)\s*</typename>").unwrap();
    let re_def = Regex::new(r"<typedef>\n((.|\n)*?)\n</typedef>").unwrap();

    // This comments out lines containing "regional" and lines that start with "Reg" followed by an uppercase letter
    // since regional extension messages are not yet supported
    let re_reg = Regex::new(r"^Reg[A-Z]|^Reg-").unwrap();
    let re_regional = Regex::new(r"\b\w*regional\b").unwrap();
    let generated = generated
        .lines()
        .map(|line| {
            if (line.contains("regional") || line.contains(" REGIONAL") || re_reg.is_match(line)) && !line.contains("<typename>"){  
                format!("# {}", line)
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");

    generated.split_inclusive("</typedef>").for_each(|s| {
        if let Some(def_caps) = re_def.captures(s) {
            let definition = def_caps.get(1).unwrap().as_str();
            if definition.trim().is_empty() {
                return;
            }
            let name = if let Some(name_caps) = re_name.captures(definition) {
                name_caps.get(2).unwrap().as_str()
            } else {
                panic!("Failed to extract message name from following definition:\n{}", definition);
            };

            // This filters out all messages starting with "Reg" followed by an uppercase letter
            // since regional extension messages are not yet supported
            if re_reg.is_match(name) || re_regional.is_match(name) {
                return;
            }

            let path = args.out.join(format!("{}.msg", name));
            std::fs::write(path, definition).unwrap();
        }
    });
}
