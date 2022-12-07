# ------------------------------------------------------------------------------
# MovementState ::= SEQUENCE {
#   movementName       DescriptiveName OPTIONAL,
#   signalGroup        SignalGroupID,    
#   state-time-speed   MovementEventList,   
#   maneuverAssistList ManeuverAssistList OPTIONAL,
#   regional           SEQUENCE (SIZE(1..4)) OF 
#                      RegionalExtension {{Reg-MovementState}} OPTIONAL,
# ------------------------------------------------------------------------------

DescriptiveName movementName
bool movementName_isPresent

SignalGroupID signalGroup

MovementEventList state_time_speed

ManeuverAssistList maneuverAssistList
bool maneuverAssistList_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
