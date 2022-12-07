# ------------------------------------------------------------------------------
# SPAT ::= SEQUENCE { 
#   timeStamp     MinuteOfTheYear OPTIONAL,  
#   name          DescriptiveName OPTIONAL, 
#   intersections IntersectionStateList,
#   regional      SEQUENCE (SIZE(1..4)) OF 
#                 RegionalExtension {{Reg-SPAT}} OPTIONAL,
# ------------------------------------------------------------------------------

MinuteOfTheYear timeStamp
bool timeStamp_isPresent

DescriptiveName name
bool name_isPresent

IntersectionStateList intersections

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
