# ------------------------------------------------------------------------------
# ComputedLane ::= SEQUENCE {
#   referenceLaneId    LaneID,
#   offsetXaxis        CHOICE {
#                         small   DrivenLineOffsetSm, 
#                         large   DrivenLineOffsetLg
#                         },  
# ------------------------------------------------------------------------------

LaneID referenceLaneId

uint8 offsetXaxis_choice

DrivenLineOffsetSm offsetXaxis_small
uint8 OFFSET_XAXIS_CHOICE_SMALL = 0
DrivenLineOffsetLg offsetXaxis_large
uint8 OFFSET_XAXIS_CHOICE_LARGE = 1

uint8 offsetYaxis_choice

DrivenLineOffsetSm offsetYaxis_small
uint8 OFFSET_YAXIS_CHOICE_SMALL = 0
DrivenLineOffsetLg offsetYaxis_large
uint8 OFFSET_YAXIS_CHOICE_LARGE = 1

# optional: True
Angle rotateXY

# optional: True
Scale_B12 scaleXaxis

# optional: True
Scale_B12 scaleYaxis

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
