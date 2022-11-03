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

# optional: True
ChargingSpotLabel chargingSpotLabel
