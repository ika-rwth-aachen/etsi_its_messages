# ------------------------------------------------------------------------------
# VehicleID ::= CHOICE {
#   entityID     TemporaryID,
#   stationID    StationID
# }
# ------------------------------------------------------------------------------

uint8 choice

TemporaryID entityID
uint8 CHOICE_ENTITY_I_D = 0
StationID stationID
uint8 CHOICE_STATION_I_D = 1
