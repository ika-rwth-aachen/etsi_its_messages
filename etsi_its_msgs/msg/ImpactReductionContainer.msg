# restricted-to: [(1, 100)]
int32 heightLonCarrLeft
int32 HEIGHT_LON_CARR_LEFT_ONE_CENTIMETER = 1
int32 HEIGHT_LON_CARR_LEFT_UNAVAILABLE = 100

# restricted-to: [(1, 100)]
int32 heightLonCarrRight
int32 HEIGHT_LON_CARR_RIGHT_ONE_CENTIMETER = 1
int32 HEIGHT_LON_CARR_RIGHT_UNAVAILABLE = 100

# restricted-to: [(1, 127)]
int32 posLonCarrLeft
int32 POS_LON_CARR_LEFT_ONE_CENTIMETER = 1
int32 POS_LON_CARR_LEFT_UNAVAILABLE = 127

# restricted-to: [(1, 127)]
int32 posLonCarrRight
int32 POS_LON_CARR_RIGHT_ONE_CENTIMETER = 1
int32 POS_LON_CARR_RIGHT_UNAVAILABLE = 127

# size: [(1, 3), None]
int32[] positionOfPillars

# restricted-to: [(1, 63)]
int32 posCentMass
int32 POS_CENT_MASS_TEN_CENTIMETERS = 1
int32 POS_CENT_MASS_UNAVAILABLE = 63

# restricted-to: [(1, 127)]
int32 wheelBaseVehicle
int32 WHEEL_BASE_VEHICLE_TEN_CENTIMETERS = 1
int32 WHEEL_BASE_VEHICLE_UNAVAILABLE = 127

# restricted-to: [(1, 255)]
int32 turningRadius
int32 TURNING_RADIUS_POINT_4_METERS = 1
int32 TURNING_RADIUS_UNAVAILABLE = 255

# restricted-to: [(1, 20)]
int32 posFrontAx
int32 POS_FRONT_AX_TEN_CENTIMETERS = 1
int32 POS_FRONT_AX_UNAVAILABLE = 20

# named-bits: [('row1LeftOccupied', '0'), ('row1RightOccupied', '1'), ('row1MidOccupied', '2'), ('row1NotDetectable', '3'), ('row1NotPresent', '4'), ('row2LeftOccupied', '5'), ('row2RightOccupied', '6'), ('row2MidOccupied', '7'), ('row2NotDetectable', '8'), ('row2NotPresent', '9'), ('row3LeftOccupied', '10'), ('row3RightOccupied', '11'), ('row3MidOccupied', '12'), ('row3NotDetectable', '13'), ('row3NotPresent', '14'), ('row4LeftOccupied', '15'), ('row4RightOccupied', '16'), ('row4MidOccupied', '17'), ('row4NotDetectable', '18'), ('row4NotPresent', '19')]
# size: [20]
string positionOfOccupants

# restricted-to: [(1, 1024)]
int32 vehicleMass
int32 VEHICLE_MASS_HUNDRED_KG = 1
int32 VEHICLE_MASS_UNAVAILABLE = 1024

int32 requestResponseIndication
int32 REQUEST_RESPONSE_INDICATION_REQUEST = 0
int32 REQUEST_RESPONSE_INDICATION_RESPONSE = 1

