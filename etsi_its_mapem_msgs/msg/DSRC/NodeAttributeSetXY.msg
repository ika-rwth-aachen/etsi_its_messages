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

NodeAttributeXYList localNode
bool localNode_isPresent

SegmentAttributeXYList disabled
bool disabled_isPresent

SegmentAttributeXYList enabled
bool enabled_isPresent

LaneDataAttributeList data
bool data_isPresent

Offset_B10 dWidth
bool dWidth_isPresent

Offset_B10 dElevation
bool dElevation_isPresent

# size: [(1, 4)]
RegionalExtension[] regional
bool regional_isPresent
