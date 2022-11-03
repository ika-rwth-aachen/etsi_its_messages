# ------------------------------------------------------------------------------
# ACKResponseService::= SEQUENCE {
#     ackRespDelayAdjust INTEGER (-32768..32767),
#     ackRespDelayStdDev INTEGER (0..65535),
#     ...
# }
# ------------------------------------------------------------------------------

# restricted-to: [(-32768, 32767)]
int16 ackRespDelayAdjust

# restricted-to: [(0, 65535)]
uint16 ackRespDelayStdDev
