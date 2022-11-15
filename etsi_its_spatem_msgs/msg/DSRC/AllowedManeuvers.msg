# ------------------------------------------------------------------------------
# AllowedManeuvers ::= BIT STRING {
#   maneuverStraightAllowed      (0), 
#   maneuverLeftAllowed          (1),  
#   maneuverRightAllowed         (2), 
#   maneuverUTurnAllowed         (3), 
#   maneuverLeftTurnOnRedAllowed (4), 
#   maneuverRightTurnOnRedAllowed (5), 
#   maneuverLaneChangeAllowed    (6), 
#   maneuverNoStoppingAllowed    (7), 
#   yieldAllwaysRequired         (8), 
#   goWithHalt                   (9), 
#   caution                      (10), 
#   reserved1                    (11)
# } (SIZE(12))
# ------------------------------------------------------------------------------

# named-bits: [('maneuverStraightAllowed', '0'), ('maneuverLeftAllowed', '1'), ('maneuverRightAllowed', '2'), ('maneuverUTurnAllowed', '3'), ('maneuverLeftTurnOnRedAllowed', '4'), ('maneuverRightTurnOnRedAllowed', '5'), ('maneuverLaneChangeAllowed', '6'), ('maneuverNoStoppingAllowed', '7'), ('yieldAllwaysRequired', '8'), ('goWithHalt', '9'), ('caution', '10'), ('reserved1', '11')]
# size: [12]
string value
