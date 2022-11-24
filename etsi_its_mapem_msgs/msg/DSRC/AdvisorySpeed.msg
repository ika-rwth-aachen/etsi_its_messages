# ------------------------------------------------------------------------------
# AdvisorySpeed ::= SEQUENCE {
#   type        AdvisorySpeedType,
#   speed       SpeedAdvice OPTIONAL,
#   confidence  SpeedConfidence OPTIONAL,
#   distance    ZoneLength OPTIONAL,
#   class       RestrictionClassID OPTIONAL,
#   regional    SEQUENCE (SIZE(1..4)) OF 
#               RegionalExtension {{Reg-AdvisorySpeed}} OPTIONAL,
# ------------------------------------------------------------------------------

AdvisorySpeedType type

# optional: True
SpeedAdvice speed

# optional: True
SpeedConfidence confidence

# optional: True
ZoneLength distance

# optional: True
RestrictionClassID class

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
