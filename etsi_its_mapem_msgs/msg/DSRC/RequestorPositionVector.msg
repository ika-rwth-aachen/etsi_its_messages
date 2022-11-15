# ------------------------------------------------------------------------------
# RequestorPositionVector ::= SEQUENCE {
#   position           Position3D,
#   heading            Angle OPTIONAL, 
#   speed              TransmissionAndSpeed OPTIONAL,
#   ...  
# }
# ------------------------------------------------------------------------------

Position3D position

# optional: True
Angle heading

# optional: True
TransmissionAndSpeed speed
