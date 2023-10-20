# ------------------------------------------------------------------------------
# YawRateValue ::= INTEGER {straight(0), degSec-000-01ToRight(-1), degSec-000-01ToLeft(1), unavailable(32767)} (-32766..32767)
# ------------------------------------------------------------------------------

int16 value
int16 MIN = -32766
int16 MAX = 32767
int16 STRAIGHT = 0
int16 DEG_SEC_000_01_TO_RIGHT = -1
int16 DEG_SEC_000_01_TO_LEFT = 1
int16 UNAVAILABLE = 32767

