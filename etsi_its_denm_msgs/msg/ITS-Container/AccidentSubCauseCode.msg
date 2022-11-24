# ------------------------------------------------------------------------------
# AccidentSubCauseCode ::= INTEGER {unavailable(0), multiVehicleAccident(1), heavyAccident(2), accidentInvolvingLorry(3), accidentInvolvingBus(4), accidentInvolvingHazardousMaterials(5), accidentOnOppositeLane(6), unsecuredAccident(7), assistanceRequested(8)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 MULTI_VEHICLE_ACCIDENT = 1
uint8 HEAVY_ACCIDENT = 2
uint8 ACCIDENT_INVOLVING_LORRY = 3
uint8 ACCIDENT_INVOLVING_BUS = 4
uint8 ACCIDENT_INVOLVING_HAZARDOUS_MATERIALS = 5
uint8 ACCIDENT_ON_OPPOSITE_LANE = 6
uint8 UNSECURED_ACCIDENT = 7
uint8 ASSISTANCE_REQUESTED = 8
uint8 value
