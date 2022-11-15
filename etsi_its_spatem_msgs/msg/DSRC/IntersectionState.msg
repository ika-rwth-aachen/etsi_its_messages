# ------------------------------------------------------------------------------
# IntersectionState ::= SEQUENCE {
#   name         DescriptiveName OPTIONAL, 
#   id           IntersectionReferenceID,  
#   revision     MsgCount,    
#   status       IntersectionStatusObject,
#   moy          MinuteOfTheYear OPTIONAL,
#   timeStamp    DSecond OPTIONAL, 
#   enabledLanes EnabledLaneList OPTIONAL,  
#   states       MovementList,
#   maneuverAssistList  ManeuverAssistList OPTIONAL,
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-IntersectionState}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
DescriptiveName name

IntersectionReferenceID id

MsgCount revision

IntersectionStatusObject status

# optional: True
MinuteOfTheYear moy

# optional: True
DSecond timeStamp

# optional: True
EnabledLaneList enabledLanes

MovementList states

# optional: True
ManeuverAssistList maneuverAssistList

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
