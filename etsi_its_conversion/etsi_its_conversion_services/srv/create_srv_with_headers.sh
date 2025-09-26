#!/bin/bash

# Check if at least one argument is provided
if [ $# -eq 0 ]; then
  echo "Usage: $0 <protocol1> <protocol2> ..."
  exit 1
fi

generated_files=()
callback_decls=()
service_members=()

for proto in "$@"; do
  # Capitalize the protocol name for filenames
  proto_cap="$(tr '[:lower:]' '[:upper:]' <<< ${proto:0:1})${proto:1}"
  proto_upper="$(echo "$proto" | tr '[:lower:]' '[:upper:]')"

  # File 1: ConvertXxxToUdp.srv
  file1="Convert${proto_cap}ToUdp.srv"
  cat > "$file1" <<EOF
etsi_its_${proto}_msgs/${proto_upper} ros_msg
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
etsi_its_${proto}_msgs/${proto_upper} ros_msg
EOF
  echo "Created $file2"
  generated_files+=("srv/$file2")

  # Callback declarations
  callback_decls+=("void udpTo${proto_cap}Callback(const std::shared_ptr<etsi_its_conversion::srv::ConvertUdpTo${proto_cap}::Request> request, std::shared_ptr<etsi_its_conversion::srv::ConvertUdpTo${proto_cap}::Response> response);")
  callback_decls+=("void ${proto}ToUdpCallback(const std::shared_ptr<etsi_its_conversion::srv::Convert${proto_cap}ToUdp::Request> request, std::shared_ptr<etsi_its_conversion::srv::Convert${proto_cap}ToUdp::Response> response);")

  # Service members
  service_members+=("rclcpp::Service<Convert${proto_cap}ToUdp>::SharedPtr convert_${proto}_to_udp_service_;")
  service_members+=("rclcpp::Service<ConvertUdpTo${proto_cap}>::SharedPtr convert_udp_to_${proto}_service_;")
done

# Print the rosidl_generate_interfaces block
echo
echo "rosidl_generate_interfaces(\${PROJECT_NAME}"
for f in "${generated_files[@]}"; do
  echo "  \"$f\""
done
echo ")"

# Print the callback declarations
echo
for decl in "${callback_decls[@]}"; do
  echo "$decl"
done

# Print the service member declarations
echo
for member in "${service_members[@]}"; do
  echo "$member"
done
