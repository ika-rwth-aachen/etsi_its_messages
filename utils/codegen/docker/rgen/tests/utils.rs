#[macro_export]
macro_rules! e2e_msgs {
    ($suite:ident, $asn1:literal, $expected:literal) => {
        #[test]
        fn $suite() {
            assert_eq!(
                rasn_compiler::Compiler::new()
                    .with_backend(ros_backend::msgs::Msgs)
                    .add_asn_literal(&format!(
                        "TestModule DEFINITIONS AUTOMATIC TAGS::= BEGIN {} END",
                        $asn1
                    ))
                    .compile_to_string()
                    .unwrap()
                    .generated
                    .replace("<typedef>\n", "")
                    .replace("\n</typedef>", "")
                    .lines()
                    .skip(1)
                    .collect::<Vec<&str>>()
                    .join("\n")
                    .replace(|c: char| c.is_whitespace(), ""),
                format!("{}", $expected)
                    .to_string()
                    .replace(|c: char| c.is_whitespace(), ""),
            )
        }
    };
}

#[macro_export]
macro_rules! e2e_hs {
    ($suite:ident, $asn1:literal, $expected:literal) => {
        #[test]
        fn $suite() {
            assert_eq!(
                rasn_compiler::Compiler::new()
                    .with_backend(
                        ros_backend::conversion::Conversion::default().set_main_pdu_name("test")
                    )
                    .add_asn_literal(&format!(
                        "TestModule DEFINITIONS AUTOMATIC TAGS::= BEGIN {} END",
                        $asn1
                    ))
                    .compile_to_string()
                    .unwrap()
                    .generated
                    .replace("#<typedef>\n", "")
                    .replace("\n#</typedef>", "")
                    .lines()
                    .skip(1)
                    .collect::<Vec<&str>>()
                    .join("\n")
                    .replace(|c: char| c.is_whitespace(), ""),
                format!("{}", $expected)
                    .to_string()
                    .replace(|c: char| c.is_whitespace(), ""),
            )
        }
    };
}
