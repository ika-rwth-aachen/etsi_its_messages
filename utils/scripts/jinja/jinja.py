#!/usr/bin/env python

from jinja2 import Environment, FileSystemLoader


asn1_definition = "multi\nline\ntest"

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
template = environment.get_template("RosMessageType.msg")
context = {
    "asn1_definition": asn1_definition,
    "members": members,
}

content = template.render(context)

with open("output/HighFrequencyContainer.msg", "w") as f:
    f.write(content)
