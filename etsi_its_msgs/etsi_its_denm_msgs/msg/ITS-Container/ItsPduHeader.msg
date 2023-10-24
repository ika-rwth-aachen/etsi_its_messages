# ------------------------------------------------------------------------------
# ItsPduHeader ::= SEQUENCE {
#     protocolVersion INTEGER (0..255),
#     messageID INTEGER{ denm(1), cam(2), poi(3), spatem(4), mapem(5), ivim(6), ev-rsr(7), tistpgtransaction(8), srem(9), ssem(10), evcsn(11), saem(12), rtcmem(13) } (0..255),  -- Mantis #7209, #7005
# ------------------------------------------------------------------------------

uint8 protocol_version
uint8 PROTOCOL_VERSION_MIN = 0
uint8 PROTOCOL_VERSION_MAX = 255

uint8 message_id
uint8 MESSAGE_ID_MIN = 0
uint8 MESSAGE_ID_MAX = 255
uint8 MESSAGE_ID_DENM = 1
uint8 MESSAGE_ID_CAM = 2
uint8 MESSAGE_ID_POI = 3
uint8 MESSAGE_ID_SPATEM = 4
uint8 MESSAGE_ID_MAPEM = 5
uint8 MESSAGE_ID_IVIM = 6
uint8 MESSAGE_ID_EV_RSR = 7
uint8 MESSAGE_ID_TISTPGTRANSACTION = 8
uint8 MESSAGE_ID_SREM = 9
uint8 MESSAGE_ID_SSEM = 10
uint8 MESSAGE_ID_EVCSN = 11
uint8 MESSAGE_ID_SAEM = 12
uint8 MESSAGE_ID_RTCMEM = 13

StationID station_id

