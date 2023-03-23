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

TimestampIts expiryTime
bool expiryTime_isPresent

Latitude protectedZoneLatitude

Longitude protectedZoneLongitude

ProtectedZoneRadius protectedZoneRadius
bool protectedZoneRadius_isPresent

ProtectedZoneID protectedZoneID
bool protectedZoneID_isPresent
