# Testing and Utility Scripts

## Testing

Test the conversion node by publishing ETSI messages in ROS, converting them to UDP bitstrings, and back to ROS messages.

### CAM Conversion

```bash
ros2 run etsi_its_conversion etsi_its_conversion_node \
    --ros-args \
        -r __node:=etsi_its_conversion \
        -r /etsi_its_conversion/udp/out:=/etsi_its_conversion/udp/in \
        -p etsi_types:=[cam] \
        -p has_btp_destination_port:=true \
        -p btp_destination_port_offset:=0 \
        -p etsi_message_payload_offset:=4
```

```bash
./publish_cam.py
```

### DENM Conversion

```bash
ros2 run etsi_its_conversion etsi_its_conversion_node \
    --ros-args \
        -r __node:=etsi_its_conversion \
        -r /etsi_its_conversion/udp/out:=/etsi_its_conversion/udp/in \
        -p etsi_types:=[denm] \
        -p has_btp_destination_port:=true \
        -p btp_destination_port_offset:=0 \
        -p etsi_message_payload_offset:=4
```

```bash
./publish_denm.py
```

### CPM (TS) Conversion

```bash
ros2 run etsi_its_conversion etsi_its_conversion_node \
    --ros-args \
        -r __node:=etsi_its_conversion \
        -r /etsi_its_conversion/udp/out:=/etsi_its_conversion/udp/in \
        -p etsi_types:=[cpm_ts] \
        -p has_btp_destination_port:=true \
        -p btp_destination_port_offset:=0 \
        -p etsi_message_payload_offset:=4
```

```bash
./publish_cpm_ts.py
```
