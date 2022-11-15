# ------------------------------------------------------------------------------
# NodeAttributeSetXY ::= SEQUENCE {
#   localNode    NodeAttributeXYList OPTIONAL,
#   disabled     SegmentAttributeXYList OPTIONAL,
#   enabled      SegmentAttributeXYList OPTIONAL,
#   data         LaneDataAttributeList OPTIONAL,
#   dWidth       Offset-B10 OPTIONAL,
#   dElevation   Offset-B10 OPTIONAL,
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-NodeAttributeSetXY}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
NodeAttributeXYList localNode

# optional: True
SegmentAttributeXYList disabled

# optional: True
SegmentAttributeXYList enabled

# optional: True
LaneDataAttributeList data

# optional: True
Offset_B10 dWidth

# optional: True
Offset_B10 dElevation

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
