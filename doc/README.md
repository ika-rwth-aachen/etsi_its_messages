# etsi_its_messages

The *etsi_its_messages* package stack allows to use standardized ETSI ITS messages for V2X communication in ROS / ROS 2 systems. Apart from the definition of ROS message equivalents to the ETSI ITS standards, this package stack also includes a conversion node for serializing the messages to and from a UDP payload, as well as RViz plugins for visualization (ROS 2 only).

All message definitions and conversion functions are automatically generated based on the [ASN.1 definitions](https://forge.etsi.org/rep/ITS/asn1) of the standardized ETSI ITS messages.

Please note that this is the Code API Documentation for the handy access functions of the generated ETSI ITS messages. Check out the [GitHub repository](https://github.com/ika-rwth-aachen/etsi_its_messages) for more information on how to install and use this package stack.