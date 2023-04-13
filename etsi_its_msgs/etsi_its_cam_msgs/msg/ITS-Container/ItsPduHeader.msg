# ------------------------------------------------------------------------------
# ItsPduHeader ::= SEQUENCE {
#     protocolVersion INTEGER (0..255),
#     messageID INTEGER{ denm(1), cam(2), poi(3), spatem(4), mapem(5), ivim(6), ev-rsr(7), tistpgtransaction(8), srem(9), ssem(10), evcsn(11), saem(12), rtcmem(13) } (0..255),  -- Mantis #7209, #7005
# ------------------------------------------------------------------------------

int64 MIN = 0
int64 MAX = 255
int64 protocolVersion

int64 MIN = 0
int64 MAX = 255
int64 MESSAGE_I_D_DENM = 1
int64 MESSAGE_I_D_CAM = 2
int64 MESSAGE_I_D_POI = 3
int64 MESSAGE_I_D_SPATEM = 4
int64 MESSAGE_I_D_MAPEM = 5
int64 MESSAGE_I_D_IVIM = 6
int64 MESSAGE_I_D_EVRSR = 7
int64 MESSAGE_I_D_TISTPGTRANSACTION = 8
int64 MESSAGE_I_D_SREM = 9
int64 MESSAGE_I_D_SSEM = 10
int64 MESSAGE_I_D_EVCSN = 11
int64 MESSAGE_I_D_SAEM = 12
int64 MESSAGE_I_D_RTCMEM = 13
int64 messageID

StationID stationID
