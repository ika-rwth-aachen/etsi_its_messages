# ------------------------------------------------------------------------------
# DangerousSituationSubCauseCode ::= INTEGER {unavailable(0), emergencyElectronicBrakeEngaged(1), preCrashSystemEngaged(2), espEngaged(3), absEngaged(4), aebEngaged(5), brakeWarningEngaged(6), collisionRiskWarningEngaged(7)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 EMERGENCY_ELECTRONIC_BRAKE_ENGAGED = 1
uint8 PRE_CRASH_SYSTEM_ENGAGED = 2
uint8 ESP_ENGAGED = 3
uint8 ABS_ENGAGED = 4
uint8 AEB_ENGAGED = 5
uint8 BRAKE_WARNING_ENGAGED = 6
uint8 COLLISION_RISK_WARNING_ENGAGED = 7
uint8 value
