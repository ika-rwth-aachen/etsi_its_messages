# ASN.1 to ROS Compiler
Backends for the rasn compiler, which converts ASN.1 files to ROS message files, and generates support conversion headers for translation between asn1c structs and ROS structs. 

Support mainly for ETSI ITS messages.

## Usage
To generate the ROS `.msg`s run `cargo run --bin asn1-to-ros-msgs -- -o <OUT> [ASN.1 files ...]`, where `<OUT>` is the output directory.

To generate the conversion headers run `cargo run --bin asn1-to-ros-conversion-headers -- -p <PDU> -o <OUT> [ASN.1 files ...]`, where `<PDU>` is the main PDU name used as a reference (e.g. `cam`, `denm`).

The corresponding ROS `.msg`s and conversion headers will be generated in the `out` directory.
