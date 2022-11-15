# ------------------------------------------------------------------------------
# RestrictionUserType ::= CHOICE {
#   basicType   RestrictionAppliesTo,
#   regional    SEQUENCE (SIZE(1..4)) OF 
#               RegionalExtension {{Reg-RestrictionUserType}}, 
# ------------------------------------------------------------------------------

uint8 choice

RestrictionAppliesTo basicType
uint8 CHOICE_BASIC_TYPE = 0
# size: [(1, 4)]
RegionalExtension[] array
uint8 CHOICE_REGIONAL = 1
