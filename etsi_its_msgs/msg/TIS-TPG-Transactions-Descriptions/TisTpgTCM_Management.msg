# ------------------------------------------------------------------------------
# TisTpgTCM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  tpgStationID StationID,
#  reservationStatus ReservationStatus,
#  reservedTpg INTEGER(1..65535)	OPTIONAL,
#  costumercontract CustomerContract OPTIONAL,
#  reservationID	ReservationID OPTIONAL,
#  tpgAutomationLevel	TpgAutomation OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

StationID tpgStationID

ReservationStatus reservationStatus

# restricted-to: [(1, 65535)]
# optional: True
uint16 reservedTpg

# optional: True
CustomerContract costumercontract

# optional: True
ReservationID reservationID

# optional: True
TpgAutomation tpgAutomationLevel
