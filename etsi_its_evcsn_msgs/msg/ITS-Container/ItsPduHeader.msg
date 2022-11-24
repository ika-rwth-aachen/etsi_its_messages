# ------------------------------------------------------------------------------
# ItsPduHeader ::= SEQUENCE {
# 	protocolVersion INTEGER{currentVersion(1)} (0..255),
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 PROTOCOL_VERSION_CURRENT_VERSION = 1
uint8 protocolVersion

# restricted-to: [(0, 255)]
uint8 MESSAGE_I_D_DENM = 1
uint8 MESSAGE_I_D_CAM = 2
uint8 MESSAGE_I_D_POI = 3
uint8 MESSAGE_I_D_SPAT = 4
uint8 MESSAGE_I_D_MAP = 5
uint8 MESSAGE_I_D_IVI = 6
uint8 MESSAGE_I_D_EVRSR = 7
uint8 messageID

StationID stationID
