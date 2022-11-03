# ------------------------------------------------------------------------------
# ITSRangingSAMAppData ::= SEQUENCE {
#     protocolVersion INTEGER (0..255),
#     groundAltitude Altitude OPTIONAL, --- From A.103 DF_Altitude from [i.21]ETSI ETSI TS 102 894 2
#     ackResponseService ACKResponseService OPTIONAL,
#     roadAngles RoadAngles OPTIONAL,
#     ... 
# }
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 protocolVersion

# optional: True
Altitude groundAltitude

# optional: True
ACKResponseService ackResponseService

# optional: True
RoadAngles roadAngles
