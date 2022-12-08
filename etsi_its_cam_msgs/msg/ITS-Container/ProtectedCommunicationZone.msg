# ------------------------------------------------------------------------------
# ProtectedCommunicationZone ::= SEQUENCE {
#     protectedZoneType       ProtectedZoneType,
#     expiryTime TimestampIts OPTIONAL,
#     protectedZoneLatitude   Latitude,
#     protectedZoneLongitude  Longitude,
#     protectedZoneRadius ProtectedZoneRadius OPTIONAL,
#     protectedZoneID ProtectedZoneID OPTIONAL,
#         ...
# }
# ------------------------------------------------------------------------------

ProtectedZoneType protectedZoneType

# optional: True
TimestampIts expiryTime
bool expiryTime_isPresent

Latitude protectedZoneLatitude

Longitude protectedZoneLongitude

# optional: True
ProtectedZoneRadius protectedZoneRadius
bool protectedZoneRadius_isPresent

# optional: True
ProtectedZoneID protectedZoneID
bool protectedZoneID_isPresent
