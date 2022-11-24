# ------------------------------------------------------------------------------
# SignalRequestPackage ::= SEQUENCE {
#   request        SignalRequest,
#   minute         MinuteOfTheYear OPTIONAL,
#   second         DSecond OPTIONAL,
#   duration       DSecond OPTIONAL,
#   regional       SEQUENCE (SIZE(1..4)) OF 
#                  RegionalExtension {{Reg-SignalRequestPackage}} OPTIONAL,
# ------------------------------------------------------------------------------

SignalRequest request

# optional: True
MinuteOfTheYear minute

# optional: True
DSecond second

# optional: True
DSecond duration

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
