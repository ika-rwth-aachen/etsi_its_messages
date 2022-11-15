# ------------------------------------------------------------------------------
# UpdateResponseCode ::= ENUMERATED { ok, unknown-Reservation-ID, mismatching-Reservation-Password, invalid-Arrival-Time, invalid-Departure-Time }
# ------------------------------------------------------------------------------

uint8 OK = 0
uint8 UNKNOWN_RESERVATION_I_D = 1
uint8 MISMATCHING_RESERVATION_PASSWORD = 2
uint8 INVALID_ARRIVAL_TIME = 3
uint8 INVALID_DEPARTURE_TIME = 4
uint8 value
