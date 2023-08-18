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

ProtectedZoneType protected_zone_type

TimestampIts expiry_time
bool expiry_time_is_present

Latitude protected_zone_latitude

Longitude protected_zone_longitude

ProtectedZoneRadius protected_zone_radius
bool protected_zone_radius_is_present

ProtectedZoneID protected_zone_i_d
bool protected_zone_i_d_is_present

