# ------------------------------------------------------------------------------
# ItsEVCSNData ::= SEQUENCE {
# 	totalNumberOfStations   NumberStations,
# 	chargingStationsData    SEQUENCE (SIZE(1..256)) OF ItsChargingStationData
# }
# ------------------------------------------------------------------------------

NumberStations totalNumberOfStations

# size: [(1, 256)]
ItsChargingStationData[] array
