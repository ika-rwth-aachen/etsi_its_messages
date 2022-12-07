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

SpeedAdvice speed
bool speed_isPresent

SpeedConfidence confidence
bool confidence_isPresent

ZoneLength distance
bool distance_isPresent

RestrictionClassID class
bool class_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
