# ------------------------------------------------------------------------------
# LaneSharing ::= BIT STRING {
#    overlappingLaneDescriptionProvided  (0),
#    multipleLanesTreatedAsOneLane       (1),
#    otherNonMotorizedTrafficTypes       (2),
#    individualMotorizedVehicleTraffic   (3),
#    busVehicleTraffic                   (4),  
#    taxiVehicleTraffic                  (5),
#    pedestriansTraffic                  (6),
#    cyclistVehicleTraffic               (7),
#    trackedVehicleTraffic               (8),
#    pedestrianTraffic                   (9)
# } (SIZE (10))
# ------------------------------------------------------------------------------

# named-bits: [('overlappingLaneDescriptionProvided', '0'), ('multipleLanesTreatedAsOneLane', '1'), ('otherNonMotorizedTrafficTypes', '2'), ('individualMotorizedVehicleTraffic', '3'), ('busVehicleTraffic', '4'), ('taxiVehicleTraffic', '5'), ('pedestriansTraffic', '6'), ('cyclistVehicleTraffic', '7'), ('trackedVehicleTraffic', '8'), ('pedestrianTraffic', '9')]
# size: [10]
string value
