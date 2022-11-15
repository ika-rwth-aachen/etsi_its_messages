# ------------------------------------------------------------------------------
# ValidityDuration ::=	INTEGER {timeOfDetection(0), oneSecondAfterDetection(1)} (0..86400)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 86400)]
uint32 TIME_OF_DETECTION = 0
uint32 ONE_SECOND_AFTER_DETECTION = 1
uint32 value
