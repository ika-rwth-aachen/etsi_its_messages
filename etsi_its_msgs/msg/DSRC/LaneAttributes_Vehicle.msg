# ------------------------------------------------------------------------------
# LaneAttributes-Vehicle ::= BIT STRING {  
#   isVehicleRevocableLane       (0),
#   isVehicleFlyOverLane         (1),
#   hovLaneUseOnly               (2),
#   restrictedToBusUse           (3),
#   restrictedToTaxiUse          (4),
#   restrictedFromPublicUse      (5),
#   hasIRbeaconCoverage          (6),
#   permissionOnRequest          (7)
# } (SIZE (8,...))
# ------------------------------------------------------------------------------

# named-bits: [('isVehicleRevocableLane', '0'), ('isVehicleFlyOverLane', '1'), ('hovLaneUseOnly', '2'), ('restrictedToBusUse', '3'), ('restrictedToTaxiUse', '4'), ('restrictedFromPublicUse', '5'), ('hasIRbeaconCoverage', '6'), ('permissionOnRequest', '7')]
# size: [8, None]
string value
