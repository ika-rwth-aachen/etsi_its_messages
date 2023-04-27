# ------------------------------------------------------------------------------
# AccelerationControl ::= BIT STRING {
#     brakePedalEngaged (0),
#     gasPedalEngaged (1),
#     emergencyBrakeEngaged (2),
#     collisionWarningEngaged (3),
#     accEngaged (4),
#     cruiseControlEngaged (5),
#     speedLimiterEngaged (6)
# } (SIZE(7))
# ------------------------------------------------------------------------------

bool[7] value
uint8 INDEX_BRAKE_PEDAL_ENGAGED = 0
uint8 INDEX_GAS_PEDAL_ENGAGED = 1
uint8 INDEX_EMERGENCY_BRAKE_ENGAGED = 2
uint8 INDEX_COLLISION_WARNING_ENGAGED = 3
uint8 INDEX_ACC_ENGAGED = 4
uint8 INDEX_CRUISE_CONTROL_ENGAGED = 5
uint8 INDEX_SPEED_LIMITER_ENGAGED = 6

