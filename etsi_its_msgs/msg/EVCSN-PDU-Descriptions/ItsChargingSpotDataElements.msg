# ------------------------------------------------------------------------------
# ItsChargingSpotDataElements ::= SEQUENCE {
# 	type			ChargingSpotType,
# 	evEquipmentID	UTF8String	OPTIONAL,
# 	typeOfReceptacle	TypeOfReceptacle,
# 	energyAvailability	UTF8String,
# 	parkingPlacesData	ParkingPlacesData	OPTIONAL
# }
# ------------------------------------------------------------------------------

ChargingSpotType type

# optional: True
string evEquipmentID

TypeOfReceptacle typeOfReceptacle

string energyAvailability

# optional: True
ParkingPlacesData parkingPlacesData
