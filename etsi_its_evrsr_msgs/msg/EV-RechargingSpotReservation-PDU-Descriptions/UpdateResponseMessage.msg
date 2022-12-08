# ------------------------------------------------------------------------------
# UpdateResponseMessage ::= SEQUENCE {
#  reservation-ID			Reservation-ID,
#  updateResponseCode	UpdateResponseCode,
#  chargingSpotLabel		ChargingSpotLabel OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

Reservation_ID reservation_ID

UpdateResponseCode updateResponseCode

ChargingSpotLabel chargingSpotLabel
bool chargingSpotLabel_isPresent
