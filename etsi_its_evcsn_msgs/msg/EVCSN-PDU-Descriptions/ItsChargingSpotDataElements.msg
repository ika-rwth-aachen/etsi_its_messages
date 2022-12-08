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

string evEquipmentID
bool evEquipmentID_isPresent

TypeOfReceptacle typeOfReceptacle

string energyAvailability

ParkingPlacesData parkingPlacesData
bool parkingPlacesData_isPresent
