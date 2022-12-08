# ------------------------------------------------------------------------------
# ClosedLanes ::= SEQUENCE {
#     innerhardShoulderStatus HardShoulderStatus OPTIONAL,
#         outerhardShoulderStatus HardShoulderStatus OPTIONAL,
#     drivingLaneStatus DrivingLaneStatus OPTIONAL,
#     ...
# }
# ------------------------------------------------------------------------------

# optional: True
HardShoulderStatus innerhardShoulderStatus
bool innerhardShoulderStatus_isPresent

# optional: True
HardShoulderStatus outerhardShoulderStatus
bool outerhardShoulderStatus_isPresent

# optional: True
DrivingLaneStatus drivingLaneStatus
bool drivingLaneStatus_isPresent
