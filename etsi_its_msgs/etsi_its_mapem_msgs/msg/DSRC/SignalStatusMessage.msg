# ------------------------------------------------------------------------------
# SignalStatusMessage ::= SEQUENCE {
#   timeStamp       MinuteOfTheYear  OPTIONAL,
#   second          DSecond,
#   sequenceNumber  MsgCount         OPTIONAL,            
#   status          SignalStatusList,
#   regional        SEQUENCE (SIZE(1..4)) OF 
#                   RegionalExtension {{Reg-SignalStatusMessage}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
MinuteOfTheYear timeStamp

DSecond second

# optional: True
MsgCount sequenceNumber

SignalStatusList status

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
