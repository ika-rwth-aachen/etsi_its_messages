# ------------------------------------------------------------------------------
# AltitudeValue ::= INTEGER {referenceEllipsoidSurface(0), oneCentimeter(1), unavailable(800001)} (-100000..800001)
# ------------------------------------------------------------------------------

# restricted-to: [(-100000, 800001)]
int32 REFERENCE_ELLIPSOID_SURFACE = 0
int32 ONE_CENTIMETER = 1
int32 UNAVAILABLE = 800001
int32 value
