use rasn_compiler::prelude::ir::IntegerType;

pub trait IntegerTypeExt {
    fn to_str(self) -> &'static str;
}

impl IntegerTypeExt for IntegerType {
    fn to_str(self) -> &'static str {
        match self {
            IntegerType::Int8 => "int8",
            IntegerType::Uint8 => "uint8",
            IntegerType::Int16 => "int16",
            IntegerType::Uint16 => "uint16",
            IntegerType::Int32 => "int32",
            IntegerType::Uint32 => "uint32",
            IntegerType::Int64 => "int64",
            IntegerType::Uint64 => "uint64",
            IntegerType::Unbounded => "int64",
        }
    }
}
pub fn to_ros_snake_case(input: &str) -> String {
    let input = input.replace('-', "_");
    let mut lowercase = String::with_capacity(input.len());

    let peekable = &mut input.chars().peekable();
    while let Some(c) = peekable.next() {
        if c.is_lowercase() || c.is_numeric() {
            lowercase.push(c);
            /* underscore before uppercase following a lowercase, aBx -> a_bx */
            if c != '_' && peekable.peek().map_or(false, |next| next.is_uppercase()) {
                lowercase.push('_');
            }
        } else {
            /* underscore before uppercase followed by lowercase, ABx -> a_bx */
            if c != '_'
                && lowercase.len() > 0
                && !lowercase.ends_with('_')
                && peekable.peek().map_or(false, |next| next.is_lowercase())
            {
                lowercase.push('_');
            }
            lowercase.push(c.to_ascii_lowercase());
        }
    }
    lowercase
}

pub fn to_ros_const_case(input: &str) -> String {
    to_ros_snake_case(input).to_string().to_uppercase()
}

pub fn to_ros_title_case(input: &str) -> String {
    if !input.is_empty() {
        let input = input.replace('-', "");
        input[0..1].to_uppercase() + &input[1..]
    } else {
        input.to_string()
    }
}

pub fn to_c_title_case(input: &str) -> String {
    let input = input.replace('-', "_");
    match input.as_str() {
        "long" => "Long".to_string(),
        "class" => "Class".to_string(),
        _ => input
    }
}
