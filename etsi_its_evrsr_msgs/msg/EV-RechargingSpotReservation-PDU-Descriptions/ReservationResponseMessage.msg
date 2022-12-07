# ------------------------------------------------------------------------------
# ReservationResponseMessage ::= SEQUENCE {
#  reservationResponseCode			ReservationResponseCode,
#  reservation-ID							Reservation-ID OPTIONAL,
#  reservation-Password					Reservation-Password OPTIONAL,
#  stationDetails							StationDetails OPTIONAL,
#  chargingSpotLabel						ChargingSpotLabel OPTIONAL,
#  expirationTime							TimestampUTC,
#  freeCancelTimeLimit					TimestampUTC OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

ReservationResponseCode reservationResponseCode

Reservation_ID reservation_ID
bool reservation_ID_isPresent

Reservation_Password reservation_Password
bool reservation_Password_isPresent

StationDetails stationDetails
bool stationDetails_isPresent

ChargingSpotLabel chargingSpotLabel
bool chargingSpotLabel_isPresent

TimestampUTC expirationTime

TimestampUTC freeCancelTimeLimit
bool freeCancelTimeLimit_isPresent
