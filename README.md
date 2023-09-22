# etsi_its_messages

TODO: BADGES

TODO

TODO: TOC


## Concept

TODO

TODO: ILLUSTRATION

### Supported ETSI ITS Messages

TODO: TABLE


## Installation

TODO

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-etsi-its-messages
```

TODO

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


## Packages

```bash
etsi_its_messages
├── etsi_its_coding
│   ├── etsi_its_coding         # metapackage including all coding packages
│   └── etsi_its_cam_coding
├── etsi_its_conversion
│   ├── etsi_its_conversion     # metapackage including all conversion packages
│   └── etsi_its_cam_conversion
├── etsi_its_messages           # metapackage including all others
├── etsi_its_msgs
│   ├── etsi_its_msgs           # metapackage including all msg packages
│   └── etsi_its_cam_msgs
└── etsi_its_rviz_plugins       # metapackage including all rviz plugin packages
    ├── etsi_its_rviz_plugins
    └── etsi_its_rviz_plugins
```

### `etsi_its_msgs`

TODO: PURPOSE, GENERATED HOW

### `etsi_its_coding`

TODO: PURPOSE, GENERATED HOW

### `etsi_its_conversion`

TODO: PURPOSE, GENERATED HOW

TODO: USAGE

TODO: DOCKER-ROS


## Acknowledgements

TODO
