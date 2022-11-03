# ------------------------------------------------------------------------------
# RescueAndRecoveryWorkInProgressSubCauseCode ::= INTEGER {unavailable(0), emergencyVehicles(1), rescueHelicopterLanding(2), policeActivityOngoing(3), medicalEmergencyOngoing(4), childAbductionInProgress(5)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 EMERGENCY_VEHICLES = 1
uint8 RESCUE_HELICOPTER_LANDING = 2
uint8 POLICE_ACTIVITY_ONGOING = 3
uint8 MEDICAL_EMERGENCY_ONGOING = 4
uint8 CHILD_ABDUCTION_IN_PROGRESS = 5
uint8 value
