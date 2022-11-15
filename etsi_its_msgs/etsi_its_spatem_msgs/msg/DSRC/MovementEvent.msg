# ------------------------------------------------------------------------------
# MovementEvent ::= SEQUENCE {
#   eventState   MovementPhaseState,
#   timing       TimeChangeDetails OPTIONAL,
#   speeds       AdvisorySpeedList OPTIONAL,
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-MovementEvent}} OPTIONAL,
# ------------------------------------------------------------------------------

MovementPhaseState eventState

# optional: True
TimeChangeDetails timing

# optional: True
AdvisorySpeedList speeds

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
