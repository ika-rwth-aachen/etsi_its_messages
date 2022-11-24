# ------------------------------------------------------------------------------
# MovementPhaseState ::= ENUMERATED {
#   unavailable (0), 
#   dark (1),   
#   stop-Then-Proceed (2), 
#   stop-And-Remain (3),
#   pre-Movement (4), 
#   permissive-Movement-Allowed (5), 
#   protected-Movement-Allowed (6), 
#   permissive-clearance (7), 
#   protected-clearance (8),  
#   caution-Conflicting-Traffic (9)
# }
# ------------------------------------------------------------------------------

uint8 UNAVAILABLE = 0
uint8 DARK = 1
uint8 STOP_THEN_PROCEED = 2
uint8 STOP_AND_REMAIN = 3
uint8 PRE_MOVEMENT = 4
uint8 PERMISSIVE_MOVEMENT_ALLOWED = 5
uint8 PROTECTED_MOVEMENT_ALLOWED = 6
uint8 PERMISSIVECLEARANCE = 7
uint8 PROTECTEDCLEARANCE = 8
uint8 CAUTION_CONFLICTING_TRAFFIC = 9
uint8 value
