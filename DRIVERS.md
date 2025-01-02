# ROS Drivers for common V2X OBU/RSU Hardware

The `etsi_its_conversion` package converts `etsi_its_msgs` ROS messages to and from [UPER-encoded](https://www.oss.com/asn1/resources/asn1-made-simple/asn1-quick-reference/packed-encoding-rules.html) [`udp_msgs/msg/UdpPacket`](https://github.com/flynneva/udp_msgs/blob/main/msg/UdpPacket.msg) payloads. This allows to build simple ROS drivers for common V2X OBUs and RSUs. This page contains instructions to connect common V2X hardware to ROS.

**Tested V2X Hardware**
- [Cohda Wireless MK5](#cohda-wireless-mk5)


## [Cohda Wireless MK5](https://www.cohdawireless.com/solutions/mk5/)

#### On the Cohda Wireless MK5

1. Download and flash the latest firmware (tested with `mk5-19.Release.139237-RSUETSI-typical`). [[Documentation](mk5-19.Release.139237-RSUETSI-typical.img)]
1. Install the `exampleETSI` application to `/mnt/rw`. [[Documentation](https://support.cohdawireless.com/hc/en-us/articles/360001755856-ExampleETSI-Installing-Running)]
1. Configure forwarding of BTP packets to the host computer via UDP in `/mnt/rw/exampleETSI/obu.conf` or `/mnt/rw/exampleETSI/rsu.conf`. [[Documentation](https://support.cohdawireless.com/hc/en-us/articles/115000972306-ETSI-Sending-receiving-BTP-packets-through-UDP)]
    ```
    ItsFacilitiesShellEnabled   = 1;     0, 1         # Enables / Disables support Facilities Shell interface
    ItsUdpBtpIfHostName     = 192.168.140.12          # hostname for sending BTP Data Ind to Shell
    ItsUdpBtpIfIndPort      = 4400;  1,65535          # port number for sending BTP Data Ind to Shell
    ItsUdpBtpIfReqPort      = 4401;  1,65535          # port number for receiving BTP Data Req from Shell
    ItsBtpShellDestPort     = 65535; 0,65535
    ```
1. *(optional)* Reduce GeoNetworking security strictness in the same file.
    ```
    ItsGnSnDecapResultHandling = 1; 0,1               # GN security strictness - STRICT (0), NON-STRICT (1)
    ```
1. Start the application or enable auto-start. [[Documentation](https://support.cohdawireless.com/hc/en-us/articles/213199623-Auto-start-an-application-after-Boot-up-of-MKx)]
    ```bash
    # root@MK5:/mnt/rw/exampleETSI#
    rc.exampleETSI start obu # or rsu
    ```

#### On the host computer

1. Install the [`udp_driver`](https://github.com/ros-drivers/transport_drivers) and [`etsi_its_conversion`](https://github.com/ika-rwth-aachen/etsi_its_messages) ROS packages.
    ```bash
    sudo apt install \
        ros-$ROS_DISTRO-transport-drivers \
        ros-$ROS_DISTRO-etsi-its-conversion
    ```
1. Configure the `udp_driver` node responsible for bridging the UDP packets received from the MK5 to [`udp_msgs/msg/UdpPacket`](https://github.com/flynneva/udp_msgs/blob/main/msg/UdpPacket.msg) ROS messages.
    ```yml
    # config.udp_driver.yml
    /**/*:
      ros__parameters:
        ip: "192.168.140.12"    # same as ItsUdpBtpIfIndHostName
        port: 4400              # same as ItsUdpBtpIfIndPort
    ```
1. Configure the `etsi_its_conversion` node responsible for converting the [UPER-encoded](https://www.oss.com/asn1/resources/asn1-made-simple/asn1-quick-reference/packed-encoding-rules.html) [`udp_msgs/msg/UdpPacket`](https://github.com/flynneva/udp_msgs/blob/main/msg/UdpPacket.msg) payloads to [`etsi_its_msgs`](https://github.com/ika-rwth-aachen/etsi_its_messages).
    ```yml
    # config.etsi_its_conversion.yml
    /**/*:
      ros__parameters:
        has_btp_destination_port: true
        btp_destination_port_offset: 8
        etsi_message_payload_offset: 78
        ros2udp_etsi_types:
          - cam
          - cam_ts
          - cpm_ts
          - denm
          - mapem_ts
          - spatem_ts
          - vam_ts
        udp2ros_etsi_types:
          - cam
          - cpm_ts
          - denm
          - mapem_ts
          - spatem_ts
          - vam_ts
    ```
1. Launch the `udp_driver` node.
    ```bash
    ros2 run udp_driver udp_receiver_node_exe --ros-args --params-file ./config.udp_driver.yml -r /udp_read:=/converter/udp/in
    ros2 lifecycle set /udp_receiver_node 1
    ros2 lifecycle set /udp_receiver_node 3
    ```
1. Receive, e.g., encoded CAMs on `/converter/udp/in`.
    ```bash
    ros2 topic echo /converter/udp/in
    ```
1. Launch the `etsi_its_conversion` node.
    ```bash
    ros2 run etsi_its_conversion etsi_its_conversion_node --ros-args --params-file ./config.etsi_its_conversion.yml
    ```
1. Receive, e.g., CAMs on `/etsi_its_conversion/cam/out`.
    ```bash
    ros2 topic echo /etsi_its_conversion/cam/out
    ```
