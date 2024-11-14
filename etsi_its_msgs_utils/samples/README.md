# Testing and Utility Scripts

## Testing

Test the conversion node by publishing ETSI messages in ROS, converting them to UDP bitstrings, and back to ROS messages.

```bash
ros2 run etsi_its_conversion etsi_its_conversion_node \
    --ros-args \
        -r __node:=etsi_its_conversion \
        -r /etsi_its_conversion/udp/out:=/etsi_its_conversion/udp/in \
        -p has_btp_destination_port:=true \
        -p btp_destination_port_offset:=0 \
        -p etsi_message_payload_offset:=4
```

```bash
./publish_cam.py
```

```bash
./publish_denm.py
```

```bash
./publish_cpm_ts.py
```

```bash
./publish_vam_ts.py
```