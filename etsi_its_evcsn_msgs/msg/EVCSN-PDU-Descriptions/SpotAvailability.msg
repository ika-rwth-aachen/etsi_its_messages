# ------------------------------------------------------------------------------
# SpotAvailability 	::= SEQUENCE {
# 	maxWaitingTimeMinutes INTEGER (0..1400), -- 0 if available or max waiting
# 						 -- time (minutes)
# 	blocking              BOOLEAN            -- true if the spot can be blocked
# }
# ------------------------------------------------------------------------------

# restricted-to: [(0, 1400)]
uint16 maxWaitingTimeMinutes

bool blocking
