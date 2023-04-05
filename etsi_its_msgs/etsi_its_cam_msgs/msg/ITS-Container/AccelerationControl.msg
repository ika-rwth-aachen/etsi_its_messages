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

# named-bits: [('brakePedalEngaged', '0'), ('gasPedalEngaged', '1'), ('emergencyBrakeEngaged', '2'), ('collisionWarningEngaged', '3'), ('accEngaged', '4'), ('cruiseControlEngaged', '5'), ('speedLimiterEngaged', '6')]
# size: [7]
uint8[] value
