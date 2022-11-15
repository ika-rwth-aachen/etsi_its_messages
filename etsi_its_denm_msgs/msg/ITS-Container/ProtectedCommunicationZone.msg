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

Latitude protectedZoneLatitude

Longitude protectedZoneLongitude

# optional: True
ProtectedZoneRadius protectedZoneRadius

# optional: True
ProtectedZoneID protectedZoneID
