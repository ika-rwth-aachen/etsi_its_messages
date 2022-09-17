int32 vehicleRole
int32 VEHICLE_ROLE_DEFAULT = 0
int32 VEHICLE_ROLE_PUBLIC_TRANSPORT = 1
int32 VEHICLE_ROLE_SPECIAL_TRANSPORT = 2
int32 VEHICLE_ROLE_DANGEROUS_GOODS = 3
int32 VEHICLE_ROLE_ROAD_WORK = 4
int32 VEHICLE_ROLE_RESCUE = 5
int32 VEHICLE_ROLE_EMERGENCY = 6
int32 VEHICLE_ROLE_SAFETY_CAR = 7
int32 VEHICLE_ROLE_AGRICULTURE = 8
int32 VEHICLE_ROLE_COMMERCIAL = 9
int32 VEHICLE_ROLE_MILITARY = 10
int32 VEHICLE_ROLE_ROAD_OPERATOR = 11
int32 VEHICLE_ROLE_TAXI = 12
int32 VEHICLE_ROLE_RESERVED_1 = 13
int32 VEHICLE_ROLE_RESERVED_2 = 14
int32 VEHICLE_ROLE_RESERVED_3 = 15

# named-bits: [('lowBeamHeadlightsOn', '0'), ('highBeamHeadlightsOn', '1'), ('leftTurnSignalOn', '2'), ('rightTurnSignalOn', '3'), ('daytimeRunningLightsOn', '4'), ('reverseLightOn', '5'), ('fogLightOn', '6'), ('parkingLightsOn', '7')]
# size: [8]
string exteriorLights

# size: [(0, 40)]
PathPoint[] pathHistory

