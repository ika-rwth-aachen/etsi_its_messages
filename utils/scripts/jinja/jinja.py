#!/usr/bin/env python

from jinja2 import Environment, FileSystemLoader


asn1_definition = "multi\nline\ntest"

etsi_type = "cam"
t_name = "HighFrequencyContainer"
t_type = "choice"

members = [
    {
        "type": "uint8",
        "name": "choice",
        "comments": [
            "restricted-to: [(0, 65535)]"
        ]
    }, {
        "type": "BasicVehicleContainerHighFrequency",
        "name": "basicVehicleContainerHighFrequency",
        "constants": [
            {
                "type": "uint8",
                "name": "CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY",
                "value": "0",
            }
        ]
    }, {
        "type": "RSUContainerHighFrequency",
        "name": "rsuContainerHighFrequency",
        "constants": [
            {
                "type": "uint8",
                "name": "CHOICE_RSU_CONTAINER_HIGH_FREQUENCY",
                "value": "1",
            }
        ],
        "optional": True
    },
]


environment = Environment(loader=FileSystemLoader("templates/"), trim_blocks=False)
template_msgs = environment.get_template("RosMessageType.msg")
template_conv = environment.get_template("ConvertFunction.h.jinja2")
context = {
    "asn1_definition": asn1_definition,
    "etsi_type": etsi_type,
    "t_type": t_type,
    "t_name": t_name,
    "members": members,
}

content_msgs = template_msgs.render(context)
content_conv = template_conv.render(context)

with open(f"output/{t_name}.msg", "w") as f:
    f.write(content_msgs)

with open(f"output/convert{t_name}.h", "w") as f:
    f.write(content_conv)
