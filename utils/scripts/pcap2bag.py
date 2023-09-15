#!/usr/bin/env python3

import argparse
from typing import List

import numpy as np
import pyshark
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from tqdm import tqdm

UDP_PACKET_MSG = """
std_msgs/Header header
string address
uint16 src_port
uint8[] data
"""
register_types(get_types_from_msg(UDP_PACKET_MSG, "udp_msgs/msg/UdpPacket"))

from rosbags.typesys.types import builtin_interfaces__msg__Time as Time
from rosbags.typesys.types import std_msgs__msg__Header as Header
from rosbags.typesys.types import udp_msgs__msg__UdpPacket as UdpPacket


def parseCli():

    parser = argparse.ArgumentParser(
        description="Extracts 802.11p ETSI ITS messages from a pcap file and stores them as udp_msgs/UdpPacket messages in a ROS bag file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument("pcap", type=str, help="pcap file")

    args = parser.parse_args()

    return args


def hexStringToUint8Array(hex_string: str) -> List[int]:

    return np.array(list(bytes.fromhex(hex_string)), dtype=np.uint8)


def main():

    args = parseCli()

    pcap = pyshark.FileCapture(args.pcap, include_raw=True, use_json=True, keep_packets=False)

    msgs = []
    for packet in tqdm(pcap):

        btp_header = hexStringToUint8Array(packet.btpb_raw.value)
        its_payload = hexStringToUint8Array(packet.its_raw.value)

        sec, nsec = (int(s) for s in packet.sniff_timestamp.split("."))
        stamp = Time(sec=sec, nanosec=nsec) # TODO: generationDeltaTime?
        header = Header(stamp=stamp, frame_id="")
        msg = UdpPacket(header=header, address="", src_port=0, data=None)
        msg.data = np.array([*btp_header, *its_payload], dtype=np.uint8)

        msgs.append(msg)

        if len(msgs) >= 4000:
            break # TODO: will otherwise crash at incomplete message at the end

    with Writer("pcap_bag") as bag:

        topic = "/etsi_its_udp"
        msg_type = UdpPacket.__msgtype__
        connection = bag.add_connection(topic, msg_type)

        for msg in tqdm(msgs):
            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            bag.write(connection, timestamp, serialize_cdr(msg, msg_type))

    exit(0)


if __name__ == "__main__":

    main()
