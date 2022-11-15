# ------------------------------------------------------------------------------
# RestrictionAppliesTo ::= ENUMERATED {
#   none,
#   equippedTransit,
#   equippedTaxis,
#   equippedOther,
#   emissionCompliant,
#   equippedBicycle,
#   weightCompliant,
#   heightCompliant,
#   pedestrians,
#   slowMovingPersons,
#   wheelchairUsers,
#   visualDisabilities,
#   audioDisabilities,
#   otherUnknownDisabilities,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 NONE = 0
uint8 EQUIPPED_TRANSIT = 1
uint8 EQUIPPED_TAXIS = 2
uint8 EQUIPPED_OTHER = 3
uint8 EMISSION_COMPLIANT = 4
uint8 EQUIPPED_BICYCLE = 5
uint8 WEIGHT_COMPLIANT = 6
uint8 HEIGHT_COMPLIANT = 7
uint8 PEDESTRIANS = 8
uint8 SLOW_MOVING_PERSONS = 9
uint8 WHEELCHAIR_USERS = 10
uint8 VISUAL_DISABILITIES = 11
uint8 AUDIO_DISABILITIES = 12
uint8 OTHER_UNKNOWN_DISABILITIES = 13
uint8 value
