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

Elevation elevation
bool elevation_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
