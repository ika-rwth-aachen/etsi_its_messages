# ------------------------------------------------------------------------------
# MovementEvent ::= SEQUENCE {
#   eventState   MovementPhaseState,
#   timing       TimeChangeDetails OPTIONAL,
#   speeds       AdvisorySpeedList OPTIONAL,
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-MovementEvent}} OPTIONAL,
# ------------------------------------------------------------------------------

MovementPhaseState eventState

TimeChangeDetails timing
bool timing_isPresent

AdvisorySpeedList speeds
bool speeds_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
