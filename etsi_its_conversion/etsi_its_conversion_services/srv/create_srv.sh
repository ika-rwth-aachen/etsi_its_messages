#!/bin/bash

# Check if at least one argument is provided
if [ $# -eq 0 ]; then
  echo "Usage: $0 <protocol1> <protocol2> ..."
  exit 1
fi

generated_files=()

for proto in "$@"; do
  # Capitalize the protocol name for filenames and message references
  proto_cap="$(tr '[:lower:]' '[:upper:]' <<< ${proto:0:1})${proto:1}"
  proto_upper="$(echo "$proto" | tr '[:lower:]' '[:upper:]')"

  # File 1: ConvertXxxToUdp.srv
  file1="Convert${proto_cap}ToUdp.srv"
  cat > "$file1" <<EOF
etsi_its_${proto}_msgs/${proto_upper} ${proto}
---
udp_msgs/UdpPacket udp_packet
EOF
  echo "Created $file1"
  generated_files+=("srv/$file1")

  # File 2: ConvertUdpToXxx.srv
  file2="ConvertUdpTo${proto_cap}.srv"
  cat > "$file2" <<EOF
udp_msgs/UdpPacket udp_packet
---
etsi_its_${proto}_msgs/${proto_upper} ${proto}
EOF
  echo "Created $file2"
  generated_files+=("srv/$file2")
done

# Print the rosidl_generate_interfaces block
echo
echo "rosidl_generate_interfaces(\${PROJECT_NAME}"
for f in "${generated_files[@]}"; do
  echo "  \"$f\""
done
echo ")"
