# ------------------------------------------------------------------------------
# YawRateValue ::= INTEGER {straight(0), degSec-000-01ToRight(-1), degSec-000-01ToLeft(1), unavailable(32767)} (-32766..32767)
# ------------------------------------------------------------------------------

int64 MIN = -32766
int64 MAX = 32767
int64 STRAIGHT = 0
int64 DEG_SEC_0_0_0_0_1_TO_RIGHT = -1
int64 DEG_SEC_0_0_0_0_1_TO_LEFT = 1
int64 UNAVAILABLE = 32767
int64 value
