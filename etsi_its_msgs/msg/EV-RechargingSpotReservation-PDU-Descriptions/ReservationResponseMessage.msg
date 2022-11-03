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

# optional: True
Reservation_ID reservation_ID

# optional: True
Reservation_Password reservation_Password

# optional: True
StationDetails stationDetails

# optional: True
ChargingSpotLabel chargingSpotLabel

TimestampUTC expirationTime

# optional: True
TimestampUTC freeCancelTimeLimit
