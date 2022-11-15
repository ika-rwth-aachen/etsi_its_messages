# ------------------------------------------------------------------------------
# MovementState ::= SEQUENCE {
#   movementName       DescriptiveName OPTIONAL,
#   signalGroup        SignalGroupID,    
#   state-time-speed   MovementEventList,   
#   maneuverAssistList ManeuverAssistList OPTIONAL,
#   regional           SEQUENCE (SIZE(1..4)) OF 
#                      RegionalExtension {{Reg-MovementState}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
DescriptiveName movementName

SignalGroupID signalGroup

MovementEventList state_time_speed

# optional: True
ManeuverAssistList maneuverAssistList

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
