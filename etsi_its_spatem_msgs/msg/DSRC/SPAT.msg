# ------------------------------------------------------------------------------
# SPAT ::= SEQUENCE { 
#   timeStamp     MinuteOfTheYear OPTIONAL,  
#   name          DescriptiveName OPTIONAL, 
#   intersections IntersectionStateList,
#   regional      SEQUENCE (SIZE(1..4)) OF 
#                 RegionalExtension {{Reg-SPAT}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
MinuteOfTheYear timeStamp

# optional: True
DescriptiveName name

IntersectionStateList intersections

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
