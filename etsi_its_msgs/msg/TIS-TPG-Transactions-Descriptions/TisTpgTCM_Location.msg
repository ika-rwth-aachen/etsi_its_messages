# ------------------------------------------------------------------------------
# TisTpgTCM-Location::= SEQUENCE {
#  tpgLocation ReferencePosition OPTIONAL,
#  address UTF8String(SIZE(1..128)) OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

# optional: True
ReferencePosition tpgLocation

# size: [(1, 128)]
# optional: True
string address
