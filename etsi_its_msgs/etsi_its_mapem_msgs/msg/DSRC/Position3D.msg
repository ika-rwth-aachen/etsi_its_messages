# ------------------------------------------------------------------------------
# Position3D ::= SEQUENCE {
#   lat        Latitude,
#   long       Longitude,
#   elevation  Elevation  OPTIONAL,
#   regional   SEQUENCE (SIZE(1..4)) OF 
#              RegionalExtension {{Reg-Position3D}} OPTIONAL,
# ------------------------------------------------------------------------------

Latitude lat

Longitude long

# optional: True
Elevation elevation

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
