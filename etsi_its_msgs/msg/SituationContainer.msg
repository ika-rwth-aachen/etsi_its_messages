# restricted-to: [(0, 7)]
int32 informationQuality
int32 INFORMATION_QUALITY_UNAVAILABLE = 0
int32 INFORMATION_QUALITY_LOWEST = 1
int32 INFORMATION_QUALITY_HIGHEST = 7

CauseCode eventType

# optional: True
CauseCode linkedCause

# size: [(1, 23)]
EventPoint[] eventHistory

