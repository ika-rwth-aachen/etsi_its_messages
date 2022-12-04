# ------------------------------------------------------------------------------
# AltitudeValue ::= INTEGER {referenceEllipsoidSurface(0), oneCentimeter(1), unavailable(800001)} (-100000..800001)
# ------------------------------------------------------------------------------

# restricted-to: [(-100000, 800001)]
int64 REFERENCE_ELLIPSOID_SURFACE = 0
int64 ONE_CENTIMETER = 1
int64 UNAVAILABLE = 800001
int64 value
