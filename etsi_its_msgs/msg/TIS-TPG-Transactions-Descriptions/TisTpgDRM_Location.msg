# ------------------------------------------------------------------------------
# TisTpgDRM-Location ::= SEQUENCE {
#  vehiclePosition ReferencePosition,
#  vehicleSpeed Speed,
#  vehicleHeading Heading,
#  requestedPosition ReferencePosition OPTIONAL,
#  searchRange SearchRange OPTIONAL,
#  searchCondition SearchCondition OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

ReferencePosition vehiclePosition

Speed vehicleSpeed

Heading vehicleHeading

# optional: True
ReferencePosition requestedPosition

# optional: True
SearchRange searchRange

# optional: True
SearchCondition searchCondition
