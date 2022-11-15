# ------------------------------------------------------------------------------
# NodeOffsetPointXY ::= CHOICE {
#   node-XY1         Node-XY-20b,
#   node-XY2         Node-XY-22b,
#   node-XY3         Node-XY-24b,
#   node-XY4         Node-XY-26b,
#   node-XY5         Node-XY-28b,
#   node-XY6         Node-XY-32b,
#   node-LatLon      Node-LLmD-64b,
#   regional         RegionalExtension {{Reg-NodeOffsetPointXY}}
# ------------------------------------------------------------------------------

uint8 choice

Node_XY_20b node_XY1
uint8 CHOICE_NODE_X_Y_1 = 0
Node_XY_22b node_XY2
uint8 CHOICE_NODE_X_Y_2 = 1
Node_XY_24b node_XY3
uint8 CHOICE_NODE_X_Y_3 = 2
Node_XY_26b node_XY4
uint8 CHOICE_NODE_X_Y_4 = 3
Node_XY_28b node_XY5
uint8 CHOICE_NODE_X_Y_5 = 4
Node_XY_32b node_XY6
uint8 CHOICE_NODE_X_Y_6 = 5
Node_LLmD_64b node_LatLon
uint8 CHOICE_NODE_LAT_LON = 6
# actual-parameters: ['{']
RegionalExtension regional
uint8 CHOICE_REGIONAL = 7
