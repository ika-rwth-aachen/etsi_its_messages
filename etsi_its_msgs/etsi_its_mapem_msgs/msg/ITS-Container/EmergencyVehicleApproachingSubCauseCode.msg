# ------------------------------------------------------------------------------
# EmergencyVehicleApproachingSubCauseCode ::= INTEGER {unavailable(0), emergencyVehicleApproaching(1), prioritizedVehicleApproaching(2)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 EMERGENCY_VEHICLE_APPROACHING = 1
uint8 PRIORITIZED_VEHICLE_APPROACHING = 2
uint8 value
