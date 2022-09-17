# named-bits: [('lightBarActivated', '0'), ('sirenActivated', '1')]
# size: [2]
string lightBarSirenInUse

# optional: True
CauseCode incidentIndication

int32 trafficRule
int32 TRAFFIC_RULE_NO_PASSING = 0
int32 TRAFFIC_RULE_NO_PASSING_FOR_TRUCKS = 1
int32 TRAFFIC_RULE_PASS_TO_RIGHT = 2
int32 TRAFFIC_RULE_PASS_TO_LEFT = 3

# restricted-to: [(1, 255)]
int32 speedLimit
int32 SPEED_LIMIT_ONE_KM_PER_HOUR = 1

