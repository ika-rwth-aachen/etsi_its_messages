# named-bits: [('lightBarActivated', '0'), ('sirenActivated', '1')]
# size: [2]
string lightBarSirenInUse

# optional: True
ClosedLanes closedLanes

# size: [(1, 3), None]
int32[] restriction

# restricted-to: [(1, 255)]
int32 speedLimit
int32 SPEED_LIMIT_ONE_KM_PER_HOUR = 1

# optional: True
CauseCode incidentIndication

# size: [(1, 40)]
ReferencePosition[] recommendedPath

# optional: True
DeltaReferencePosition startingPointSpeedLimit

int32 trafficFlowRule
int32 TRAFFIC_FLOW_RULE_NO_PASSING = 0
int32 TRAFFIC_FLOW_RULE_NO_PASSING_FOR_TRUCKS = 1
int32 TRAFFIC_FLOW_RULE_PASS_TO_RIGHT = 2
int32 TRAFFIC_FLOW_RULE_PASS_TO_LEFT = 3

# size: [(1, 8), None]
ActionID[] referenceDenms

