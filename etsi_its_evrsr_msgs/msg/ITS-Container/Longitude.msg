# ------------------------------------------------------------------------------
# Longitude ::= INTEGER {oneMicrodegreeEast (10), oneMicrodegreeWest (-10), unavailable(1800000001)} (-1800000000..1800000001)
# ------------------------------------------------------------------------------

# restricted-to: [(-1800000000, 1800000001)]
int32 ONE_MICRODEGREE_EAST = 10
int32 ONE_MICRODEGREE_WEST = -10
int32 UNAVAILABLE = 1800000001
int32 value
