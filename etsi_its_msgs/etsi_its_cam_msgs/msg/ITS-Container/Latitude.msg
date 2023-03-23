# ------------------------------------------------------------------------------
# Latitude ::= INTEGER {oneMicrodegreeNorth (10), oneMicrodegreeSouth (-10), unavailable(900000001)} (-900000000..900000001)
# ------------------------------------------------------------------------------

# restricted-to: [(-900000000, 900000001)]
int64 ONE_MICRODEGREE_NORTH = 10
int64 ONE_MICRODEGREE_SOUTH = -10
int64 UNAVAILABLE = 900000001
int64 value
