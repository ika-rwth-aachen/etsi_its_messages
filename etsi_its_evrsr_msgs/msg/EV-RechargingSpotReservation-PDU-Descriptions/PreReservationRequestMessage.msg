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

TimestampUTC departureTime
bool departureTime_isPresent

RechargingType rechargingType

BatteryType batteryType
bool batteryType_isPresent
