# etsi_its_messages

<p align="center">
  <img src="https://img.shields.io/github/v/release/ika-rwth-aachen/etsi_its_messages"/></a>
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/etsi_its_messages"/>
  <a href="https://github.com/ika-rwth-aachen/etsi_its_messages/actions/workflows/codegen.yml"><img src="https://github.com/ika-rwth-aachen/etsi_its_messages/actions/workflows/codegen.yml/badge.svg"/></a>
  <a href="https://github.com/ika-rwth-aachen/etsi_its_messages/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/etsi_its_messages/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS-noetic-blueviolet"/>
  <img src="https://img.shields.io/badge/ROS 2-humble|iron|rolling-blueviolet"/>
</p>

**ROS / ROS 2 Support for ETSI ITS Messages for V2X Communication**

The *etsi_its_messages* package stack allows to use standardized ETSI ITS messages for V2X communicaiton in ROS / ROS 2 systems. Apart from the definition of ROS message equivalents to the ETSI ITS standards, this package stack also includes a conversion node for serializing the messages to and from a UDP payload, as well as RViz plugins for visualization (ROS 2 only).

- [Concept](#concept)
- [Supported ETSI ITS Messages](#supported-etsi-its-messages)
- [Packages](#packages)
  - [`etsi_its_msgs`](#etsi-its-msgs)
  - [`etsi_its_coding`](#etsi-its-coding)
  - [`etsi_its_conversion`](#etsi-its-conversion)
- [Installation](#installation)
  - [docker-ros](#docker-ros)


## Concept

TODO

TODO: ILLUSTRATION


## Supported ETSI ITS Messages

| Status | Acronym | Name | Version | Definition |
| --- | --- | --- | --- | --- |
| :white_check_mark: | CAM | Cooperative Awareness Message | TODO | [Link](TODO) |
| :white_check_mark: | DENM | Decentralized Environmental Notification Message | TODO | [Link](TODO) |
| :soon: | MAPEM | Map Extended Message | - | - |
| :soon: | SPATEM | Signal Phase and Timing Extended Message | - | - |
| :soon: | CPM | Collective Perception Message | - | - |


## Packages

```bash
etsi_its_messages
├── etsi_its_coding
│   ├── etsi_its_coding         # metapackage including all coding packages
│   ├── etsi_its_cam_coding
│   └── etsi_its_cam_coding
├── etsi_its_conversion
│   ├── etsi_its_conversion     # conversion node depending on all conversion packages
│   ├── etsi_its_cam_conversion
│   ├── etsi_its_denm_conversion
│   └── etsi_its_primitives_conversion
├── etsi_its_messages           # metapackage including all others
├── etsi_its_msgs
│   ├── etsi_its_msgs           # metapackage including all msg packages
│   ├── etsi_its_cam_msgs
│   └── etsi_its_denm_msgs
└── etsi_its_rviz_plugins
```

### `etsi_its_msgs`

The `etsi_its_msgs` metapackage includes one dedicated package for each ETSI ITS message type, e.g., `etsi_its_cam_msgs`. These packages define the ROS message equivalents to the ETSI ITS message types, e.g., [`etsi_its_cam_msgs/msg/CAM`](etsi_its_msgs/etsi_its_cam_msgs/msg/CAM-PDU-Descriptions/CAM.msg).

In addition, the `etsi_its_msgs` contains header-only libraries providing helpful access functions for modifying the deeply nested ROS messages.

#### Automated Generation

The ROS message files are auto-generated based on the ASN.1 definition of the ETSI ITS message standards.

```bash
# etsi_its_messages$
./utils/codegen/scripts/asn1ToRosMsg.py \
  asn1/reduced/cam/CAM-PDU-Descriptions.asn \
  asn1/reduced/cam/ITS-Container.asn \
  -o etsi_its_msgs/etsi_its_cam_msgs/msg
```

### `etsi_its_coding`

The `etsi_its_coding` metapackage includes one dedicated package for each ETSI ITS message type, e.g., `etsi_its_cam_coding`. These packages provide C++ libraries containing a `struct` implementation of the ETSI ITS message types including functions for encoding and decoding the structures to binary buffers.

#### Automated Generation

The C/C++ implementation of the message types is auto-generated based on the ASN.1 definition of the ETSI ITS message standards, using the [ASN.1 Compiler asn1c](https://github.com/vlm/asn1c).

```bash
# etsi_its_messages$
./utils/codegen/scripts/asn1ToC.py
  asn1/reduced/cam/CAM-PDU-Descriptions.asn \
  asn1/reduced/cam/ITS-Container.asn \
  -o etsi_its_coding/etsi_its_cam_coding
```

### `etsi_its_conversion`

The `etsi_its_conversion` package provides a C++ ROS nodelet or ROS 2 component node for converting `etsi_its_msgs` ROS messages to and from [PER-encoded](https://www.oss.com/asn1/resources/asn1-made-simple/asn1-quick-reference/packed-encoding-rules.html) [`udp_msgs/msg/UdpPacket`](https://github.com/flynneva/udp_msgs/blob/main/msg/UdpPacket.msg) payloads. This way, ETSI ITS messages cannot only be used within the ROS ecosystem, but may also be received from or sent to outside applications. (TODO: PER-encoding?)

The package depends on one dedicated package for each ETSI ITS message type, e.g., `etsi_its_cam_conversion`. These packages hold header-only libraries with recursive conversion functions for each nested message type.

#### Usage

The conversion node bridges all ETSI ITS message types at the same time in both directions.

```bash
# ROS 2
ros2 launch etsi_its_conversion converter.launch.py
# or
ros2 run etsi_its_conversion etsi_its_conversion_node --ros-args -p etsi_type:=auto

# ROS
roslaunch etsi_its_conversion converter.ros1.launch
# or
rosrun nodelet nodelet standalone etsi_its_conversion/Converter _etsi_type:=auto
```

##### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/udp/in` | `udp_msgs/msg/UdpPacket` | UDP payload for conversion to ROS |
| `~/cam/in` | `etsi_its_cam_msgs/msg/CAM` | CAM for conversion to UDP |
| `~/denm/in` | `etsi_its_denm_msgs/msg/DENM` | DENM for conversion to UDP |

##### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/udp/out` | `udp_msgs/msg/UdpPacket` | UDP payload converted from ROS message |
| `~/cam/out` | `etsi_its_cam_msgs/msg/CAM` | CAM converted from UDP payload |
| `~/denm/out` | `etsi_its_denm_msgs/msg/DENM` | DENM converted from UDP payload |

##### Parameters

| Parameter | Type | Description | Options |
| --- | --- | --- | --- |
| `etsi_type` | `string` | if set to `auto`, in- and outgoing UDP payloads are expected to include a [4-byte BTP header](https://www.etsi.org/deliver/etsi_en/302600_302699/3026360501/02.01.00_20/en_3026360501v020100a.pdf) | `auto`, `cam`, `denm` |


#### Automated Generation

The C++ conversion functions are auto-generated based on the ASN.1 definition of the ETSI ITS message standards.

```bash
# etsi_its_messages$
./utils/codegen/scripts/asn1ToConversionHeader.py
  asn1/reduced/cam/CAM-PDU-Descriptions.asn \
  asn1/reduced/cam/ITS-Container.asn \
  -t cam \
  -o etsi_its_conversion/etsi_its_cam_conversion/include/etsi_its_cam_conversion
```


## Installation

All *etsi_its_messages* packages are released as official ROS / ROS 2 packages and can easily be installed via a package manager.

> [!WARNING]  
> The initial release may not have been synced to the package managers yet. In the meantime, please refer to installation from source as shown below.

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-etsi-its-messages
```

If you would like to install *etsi_its_messages* from source, simply clone this repository into your ROS workspace. All dependencies that are listed in the packages' `package.xml` can be installed using [*rosdep*](http://wiki.ros.org/rosdep).

```bash
# etsi_its_messages$
rosdep install -r --ignore-src --from-paths .

# ROS 2
# workspace$
colcon build --packages-up-to etsi_its_messages --cmake-args -DCMAKE_BUILD_TYPE=Release

# ROS
# workspace$
catkin build -DCMAKE_BUILD_TYPE=Release etsi_its_messages
```

### docker-ros

The *etsi_its_messages* package stack is also available as a Docker image, containerized through [*docker-ros*](https://github.com/ika-rwth-aachen/docker-ros). Note that launching these containers starts the `etsi_its_conversion` node by default.

```bash
# ROS 2
docker run --rm ghcr.io/ika-rwth-aachen/etsi_its_messages:ros2

# ROS
docker run --rm ghcr.io/ika-rwth-aachen/etsi_its_messages:ros
```


## Acknowledgements

This work is accomplished within the projects 6GEM (FKZ 16KISK036K), AUTOtech.agil (FKZ 01IS22088A), and AIthena (TODO). We acknowledge the financial support for the projects by the Federal Ministry of Education and Research of Germany (BMBF) and TODO.
