# ------------------------------------------------------------------------------
# ExteriorLights ::= BIT STRING {
#     lowBeamHeadlightsOn      (0),
#     highBeamHeadlightsOn     (1),
#     leftTurnSignalOn         (2),
#     rightTurnSignalOn        (3),
#     daytimeRunningLightsOn   (4),
#     reverseLightOn           (5),
#     fogLightOn               (6),
#     parkingLightsOn          (7)
# } (SIZE(8))
# ------------------------------------------------------------------------------

bool[8] value
uint8 INDEX_LOW_BEAM_HEADLIGHTS_ON = 0
uint8 INDEX_HIGH_BEAM_HEADLIGHTS_ON = 1
uint8 INDEX_LEFT_TURN_SIGNAL_ON = 2
uint8 INDEX_RIGHT_TURN_SIGNAL_ON = 3
uint8 INDEX_DAYTIME_RUNNING_LIGHTS_ON = 4
uint8 INDEX_REVERSE_LIGHT_ON = 5
uint8 INDEX_FOG_LIGHT_ON = 6
uint8 INDEX_PARKING_LIGHTS_ON = 7

