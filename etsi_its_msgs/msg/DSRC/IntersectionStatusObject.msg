# ------------------------------------------------------------------------------
# IntersectionStatusObject ::= BIT STRING {
#   manualControlIsEnabled                (0),
#   stopTimeIsActivated                   (1),
#   failureFlash                          (2), 
#   preemptIsActive                       (3),
#   signalPriorityIsActive                (4),   
#   fixedTimeOperation                    (5),
#   trafficDependentOperation             (6),
#   standbyOperation                      (7),
#   failureMode                           (8),
#   off                                   (9),
#   recentMAPmessageUpdate                (10),
#   recentChangeInMAPassignedLanesIDsUsed (11),
#   noValidMAPisAvailableAtThisTime       (12),
#   noValidSPATisAvailableAtThisTime      (13)
# } (SIZE(16))
# ------------------------------------------------------------------------------

# named-bits: [('manualControlIsEnabled', '0'), ('stopTimeIsActivated', '1'), ('failureFlash', '2'), ('preemptIsActive', '3'), ('signalPriorityIsActive', '4'), ('fixedTimeOperation', '5'), ('trafficDependentOperation', '6'), ('standbyOperation', '7'), ('failureMode', '8'), ('off', '9'), ('recentMAPmessageUpdate', '10'), ('recentChangeInMAPassignedLanesIDsUsed', '11'), ('noValidMAPisAvailableAtThisTime', '12'), ('noValidSPATisAvailableAtThisTime', '13')]
# size: [16]
string value
