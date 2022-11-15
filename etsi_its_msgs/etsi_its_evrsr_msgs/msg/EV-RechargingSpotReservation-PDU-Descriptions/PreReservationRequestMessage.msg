# ------------------------------------------------------------------------------
# PreReservationRequestMessage ::= SEQUENCE {
#  evse-ID 			EVSE-ID,
#  arrivalTime		TimestampUTC,
#  departureTime	TimestampUTC OPTIONAL,
#  rechargingType	RechargingType,
#  batteryType		BatteryType OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

EVSE_ID evse_ID

TimestampUTC arrivalTime

# optional: True
TimestampUTC departureTime

RechargingType rechargingType

# optional: True
BatteryType batteryType
