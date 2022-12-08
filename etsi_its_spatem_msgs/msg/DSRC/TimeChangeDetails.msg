# ------------------------------------------------------------------------------
# TimeChangeDetails ::= SEQUENCE {
#   startTime   TimeMark           OPTIONAL,
#   minEndTime  TimeMark,
#   maxEndTime  TimeMark           OPTIONAL,
#   likelyTime  TimeMark           OPTIONAL,
#   confidence  TimeIntervalConfidence OPTIONAL,
#   nextTime    TimeMark           OPTIONAL
# }
# ------------------------------------------------------------------------------

TimeMark startTime
bool startTime_isPresent

TimeMark minEndTime

TimeMark maxEndTime
bool maxEndTime_isPresent

TimeMark likelyTime
bool likelyTime_isPresent

TimeIntervalConfidence confidence
bool confidence_isPresent

TimeMark nextTime
bool nextTime_isPresent
