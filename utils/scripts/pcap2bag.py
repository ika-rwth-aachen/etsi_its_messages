#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import argparse
import os
import shutil

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
        description="Extracts 802.11p ETSI ITS messages from a pcap file and stores them as 'udp_msgs/msg/UdpPacket' messages in a ROS bag file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument("pcap", type=str, help="pcap file")
    parser.add_argument("-o", "--output-bag", type=str, default=None, help="output bag directory")
    parser.add_argument("-f", "--force", action="store_true", default=False, help="overwrite existing output bag")
    parser.add_argument("-t", "--topic", type=str, default="/etsi_its_udp", help="topic name for 'udp_msgs/msg/UdpPacket' messages")

    args = parser.parse_args()

    if args.output_bag is None:
        args.output_bag = os.path.splitext(args.pcap)[0] + "_bag"

    return args


def hexStringToUint8Array(hex_string: str) -> np.ndarray:

    return np.array(list(bytes.fromhex(hex_string)), dtype=np.uint8)


def main():

    args = parseCli()

    # parse packets from pcap
    print(f"Loading packets from {args.pcap} ...", end="", flush=True)
    pcap = pyshark.FileCapture("rx_r1a.pcap", include_raw=True, use_json=True)
    pcap.load_packets()
    print(" done")

    # convert packets to ROS messages
    msgs = []
    for packet in tqdm(pcap, total=len(pcap), desc="Converting packets to ROS messages"):

        btp_header = hexStringToUint8Array(packet.btpb_raw.value)
        its_payload = hexStringToUint8Array(packet.its_raw.value)

        sec, nsec = (int(s) for s in packet.sniff_timestamp.split("."))
        stamp = Time(sec=sec, nanosec=nsec)
        header = Header(stamp=stamp, frame_id="")
        msg = UdpPacket(header=header, address="", src_port=0, data=None)
        msg.data = np.array([*btp_header, *its_payload], dtype=np.uint8)

        msgs.append(msg)

    # write ROS messages to bag
    if os.path.exists(args.output_bag) and args.force:
        shutil.rmtree(args.output_bag)
    with Writer(args.output_bag) as bag:

        topic = args.topic
        msg_type = UdpPacket.__msgtype__
        connection = bag.add_connection(topic, msg_type)

        for msg in tqdm(msgs, desc=f"Writing ROS messages to {args.output_bag}"):
            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            bag.write(connection, timestamp, serialize_cdr(msg, msg_type))


if __name__ == "__main__":

    main()
