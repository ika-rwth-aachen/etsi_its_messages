# ------------------------------------------------------------------------------
# TisTpgTRM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  vehicleType UNVehicleClassifcation,
#  tpgStationID StationID, 
#  reservationStatus ReservationStatus,
#  costumercontract CustomerContract OPTIONAL,
#  reservationID ReservationID OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

UNVehicleClassifcation vehicleType

StationID tpgStationID

ReservationStatus reservationStatus

# optional: True
CustomerContract costumercontract

# optional: True
ReservationID reservationID
