# ------------------------------------------------------------------------------
# FullPositionVector ::= SEQUENCE {
#    utcTime             DDateTime OPTIONAL,   -- time with mSec precision
#    long                Longitude,
#    lat                 Latitude,
#    elevation           Elevation  OPTIONAL,
#    heading             Heading OPTIONAL, 
#    speed               TransmissionAndSpeed OPTIONAL,
#    posAccuracy         PositionalAccuracy OPTIONAL,
#    timeConfidence      TimeConfidence OPTIONAL,
#    posConfidence       PositionConfidenceSet OPTIONAL,
#    speedConfidence     SpeedandHeadingandThrottleConfidence OPTIONAL, 
#    ...  
#    }
# ------------------------------------------------------------------------------

# optional: True
DDateTime utcTime

Longitude long

Latitude lat

# optional: True
Elevation elevation

# optional: True
Heading heading

# optional: True
TransmissionAndSpeed speed

# optional: True
PositionalAccuracy posAccuracy

# optional: True
TimeConfidence timeConfidence

# optional: True
PositionConfidenceSet posConfidence

# optional: True
SpeedandHeadingandThrottleConfidence speedConfidence
