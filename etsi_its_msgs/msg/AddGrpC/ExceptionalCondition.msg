# ------------------------------------------------------------------------------
# ExceptionalCondition ::= ENUMERATED {
#   unknown,
#   publicTransportPriority,
#   emergencyVehiclePriority,
#   trainPriority,
#   bridgeOpen,
#   vehicleHeight,
#   weather,
#   trafficJam,
#   tunnelClosure,
#   meteringActive,
#   truckPriority,
#   bicyclePlatoonPriority,
#   vehiclePlatoonPriority,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 UNKNOWN = 0
uint8 PUBLIC_TRANSPORT_PRIORITY = 1
uint8 EMERGENCY_VEHICLE_PRIORITY = 2
uint8 TRAIN_PRIORITY = 3
uint8 BRIDGE_OPEN = 4
uint8 VEHICLE_HEIGHT = 5
uint8 WEATHER = 6
uint8 TRAFFIC_JAM = 7
uint8 TUNNEL_CLOSURE = 8
uint8 METERING_ACTIVE = 9
uint8 TRUCK_PRIORITY = 10
uint8 BICYCLE_PLATOON_PRIORITY = 11
uint8 VEHICLE_PLATOON_PRIORITY = 12
uint8 value
