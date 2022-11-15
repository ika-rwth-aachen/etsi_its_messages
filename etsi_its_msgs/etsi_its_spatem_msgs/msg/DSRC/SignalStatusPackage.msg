# ------------------------------------------------------------------------------
# SignalStatusPackage ::= SEQUENCE {
#   requester    SignalRequesterInfo OPTIONAL,
#   inboundOn    IntersectionAccessPoint,
#   outboundOn   IntersectionAccessPoint OPTIONAL,
# 
#   minute       MinuteOfTheYear OPTIONAL,
#   second       DSecond OPTIONAL,
#   duration     DSecond OPTIONAL,
#   status       PrioritizationResponseStatus,
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-SignalStatusPackage}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
SignalRequesterInfo requester

IntersectionAccessPoint inboundOn

# optional: True
IntersectionAccessPoint outboundOn

# optional: True
MinuteOfTheYear minute

# optional: True
DSecond second

# optional: True
DSecond duration

PrioritizationResponseStatus status

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
