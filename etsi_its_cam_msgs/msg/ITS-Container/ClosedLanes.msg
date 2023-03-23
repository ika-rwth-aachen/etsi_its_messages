# ------------------------------------------------------------------------------
# ClosedLanes ::= SEQUENCE {
#     innerhardShoulderStatus HardShoulderStatus OPTIONAL,
#         outerhardShoulderStatus HardShoulderStatus OPTIONAL,
#     drivingLaneStatus DrivingLaneStatus OPTIONAL,
#     ...
# }
# ------------------------------------------------------------------------------

HardShoulderStatus innerhardShoulderStatus
bool innerhardShoulderStatus_isPresent

HardShoulderStatus outerhardShoulderStatus
bool outerhardShoulderStatus_isPresent

DrivingLaneStatus drivingLaneStatus
bool drivingLaneStatus_isPresent
