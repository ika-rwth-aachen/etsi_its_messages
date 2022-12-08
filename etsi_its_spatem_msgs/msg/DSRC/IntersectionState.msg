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

DescriptiveName name
bool name_isPresent

IntersectionReferenceID id

MsgCount revision

IntersectionStatusObject status

MinuteOfTheYear moy
bool moy_isPresent

DSecond timeStamp
bool timeStamp_isPresent

EnabledLaneList enabledLanes
bool enabledLanes_isPresent

MovementList states

ManeuverAssistList maneuverAssistList
bool maneuverAssistList_isPresent

# size: [(1, 4)]
RegionalExtension[] regional
bool regional_isPresent
