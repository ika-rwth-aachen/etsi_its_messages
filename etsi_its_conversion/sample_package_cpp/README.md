# sample_package_cpp

The `sample_package_cpp` contains a simple ROS Node and a simple ROS Nodelet that can publish and subscribe a dummy topic for testing purposes.

> **Note**
> **Deprecated**: Try to use the [ROS 2 samples](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/templates/ros2/sample_package_cpp) if possible.

> **Note**
> This README is based on the `docker-ros` [README template](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-ros/-/blob/main/templates/README.template.md).

> **Note**
> This repository contains a [`.gitlab-ci.yml`](./.gitlab-ci.yml) file that defines a CI pipeline based on [docker-ros](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-ros) and [ros-industrial/industrial_ci](https://github.com/ros-industrial/industrial_ci). In addition, the pipeline provides sample docker images.


- [Nodes](#nodes)
  - [sample_package_cpp/SampleNode](#sample_package_samplenode)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Available Images](#available-images)
  - [Default Command](#default-command)
  - [Environment Variables](#environment-variables)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)
  - [Additional Remarks](#additional-remarks)
- [Official Documentation](#official-documentation)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `sample_package_cpp` | `SampleNode` | Can be used as a template for ROS 1 Nodes. Contains a dummy publisher and subscriber that can be (de-)activated individually. |
| `sample_package_cpp` | `SampleNodelet` | Can be used as a template for ROS 1 Nodelets. Contains a dummy publisher and subscriber that can be (de-)activated individually. |

### sample_package_cpp/SampleNode

> **Note**
> Should be identical for sample_package_cpp/SampleNodelet.

#### Subscribed Topics

| Topic | Type | Description | 
| --- | --- | --- |
| `~/message` | [`std_msgs/msg/String`](https://github.com/ros/std_msgs/blob/kinetic-devel/msg/String.msg) | Receives "SampleNode is running" |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/message` | [`std_msgs/msg/String`](https://github.com/ros/std_msgs/blob/kinetic-devel/msg/String.msg) | Publishes "SampleNode is running" |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `create_publisher` | `bool` | whether to create a publisher (`true`) |
| `create_subscriber` | `bool` | whether to create a subscriber (`true`) |
| `parameter_float` | `double_t` | floating point number parameter" |
| `parameter_bool` | `bool_t` | boolean parameter |
| `parameter_string` | `str_t` | string parameter |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `gitlab.ika.rwth-aachen.de:5050/fb-fi/ops/templates/ros1/sample_package_cpp:latest` | latest version (based on ROS noetic) |

### Default Command

```bash
roslaunch sample_package_cpp start_SampleNode.launch 
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `sample_package_cpp` | `start_SampleNode.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNode.launch` | Creates both a publisher and a subscriber |
| `sample_package_cpp` | `start_SampleNode.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNode.launch create_publisher:=false` | Creates only a subscriber |
| `sample_package_cpp` | `start_SampleNode.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNode.launch create_subscriber:=false` | Creates only a publisher |
| `sample_package_cpp` | `start_SampleNodelet.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNodelet.launch` | Creates both a publisher and a subscriber |
| `sample_package_cpp` | `start_SampleNodelet.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNodelet.launch create_publisher:=false` | Creates only a subscriber |
| `sample_package_cpp` | `start_SampleNodelet.launch` | `/docker-ros/ws/install/share/sample_package_cpp/launch/start_SampleNodelet.launch create_subscriber:=false` | Creates only a publisher |

### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `sample_package_cpp` | `params_SampleNode.yaml` | `/docker-ros/ws/install/share/sample_package_cpp/launch/params_SampleNode.yaml` | Contains dummy parameters. |
| `sample_package_cpp` | `params_SampleNodelet.yaml` | `/docker-ros/ws/install/share/sample_package_cpp/launch/params_SampleNodelet.yaml` | Contains dummy parameters. |

### Additional Remarks


## Official Documentation

