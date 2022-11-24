# ------------------------------------------------------------------------------
# SignalRequestMessage ::= SEQUENCE {    
#   timeStamp       MinuteOfTheYear  OPTIONAL,
#   second          DSecond,
#   sequenceNumber  MsgCount         OPTIONAL,
#   requests        SignalRequestList OPTIONAL,
#   requestor       RequestorDescription,
#   regional        SEQUENCE (SIZE(1..4)) OF 
#                   RegionalExtension {{Reg-SignalRequestMessage}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
MinuteOfTheYear timeStamp

DSecond second

# optional: True
MsgCount sequenceNumber

# optional: True
SignalRequestList requests

RequestorDescription requestor

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
