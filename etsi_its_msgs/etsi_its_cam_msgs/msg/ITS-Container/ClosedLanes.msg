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

# optional: True
HardShoulderStatus outerhardShoulderStatus

# optional: True
DrivingLaneStatus drivingLaneStatus
