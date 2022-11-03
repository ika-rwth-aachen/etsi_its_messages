# ------------------------------------------------------------------------------
# RTCMcorrections ::= SEQUENCE {
#    msgCnt      MsgCount, 
#    rev         RTCM-Revision,
#    timeStamp   MinuteOfTheYear  OPTIONAL,
#    anchorPoint FullPositionVector OPTIONAL,
#    rtcmHeader  RTCMheader OPTIONAL,  
#    msgs        RTCMmessageList,
#    regional    SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-RTCMcorrections}} OPTIONAL,
# ------------------------------------------------------------------------------

MsgCount msgCnt

RTCM_Revision rev

# optional: True
MinuteOfTheYear timeStamp

# optional: True
FullPositionVector anchorPoint

# optional: True
RTCMheader rtcmHeader

RTCMmessageList msgs

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
