# ------------------------------------------------------------------------------
# StatusResponseCode ::= ENUMERATED { charging-Spot-Occupied, charging-Spot-Unoccupied, charging-Ongoing, reservation-Limit-Exceeded, reservation-Cancelled, unknown-Reservation-ID, expired-Reservation-ID }
# ------------------------------------------------------------------------------

uint8 CHARGING_SPOT_OCCUPIED = 0
uint8 CHARGING_SPOT_UNOCCUPIED = 1
uint8 CHARGING_ONGOING = 2
uint8 RESERVATION_LIMIT_EXCEEDED = 3
uint8 RESERVATION_CANCELLED = 4
uint8 UNKNOWN_RESERVATION_I_D = 5
uint8 EXPIRED_RESERVATION_I_D = 6
uint8 value
