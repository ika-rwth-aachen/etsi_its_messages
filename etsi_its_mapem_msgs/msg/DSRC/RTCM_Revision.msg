# ------------------------------------------------------------------------------
# RTCM-Revision ::= ENUMERATED {
#   unknown (0),
#   rtcmRev2 (1), -- Std 10402.x et al
#   rtcmRev3 (2), -- Std 10403.x et al
#   reserved (3),
#   ...
# }
# ------------------------------------------------------------------------------

uint8 UNKNOWN = 0
uint8 RTCM_REV_2 = 1
uint8 RTCM_REV_3 = 2
uint8 RESERVED = 3
uint8 value
