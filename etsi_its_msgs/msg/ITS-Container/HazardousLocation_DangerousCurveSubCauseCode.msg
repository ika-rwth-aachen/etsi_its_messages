# ------------------------------------------------------------------------------
# HazardousLocation-DangerousCurveSubCauseCode ::= INTEGER {unavailable(0), dangerousLeftTurnCurve(1), dangerousRightTurnCurve(2), multipleCurvesStartingWithUnknownTurningDirection(3), multipleCurvesStartingWithLeftTurn(4), multipleCurvesStartingWithRightTurn(5)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 DANGEROUS_LEFT_TURN_CURVE = 1
uint8 DANGEROUS_RIGHT_TURN_CURVE = 2
uint8 MULTIPLE_CURVES_STARTING_WITH_UNKNOWN_TURNING_DIRECTION = 3
uint8 MULTIPLE_CURVES_STARTING_WITH_LEFT_TURN = 4
uint8 MULTIPLE_CURVES_STARTING_WITH_RIGHT_TURN = 5
uint8 value
