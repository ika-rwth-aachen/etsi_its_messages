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

# named-bits: [('lowBeamHeadlightsOn', '0'), ('highBeamHeadlightsOn', '1'), ('leftTurnSignalOn', '2'), ('rightTurnSignalOn', '3'), ('daytimeRunningLightsOn', '4'), ('reverseLightOn', '5'), ('fogLightOn', '6'), ('parkingLightsOn', '7')]
# size: [8]
uint8[] value
