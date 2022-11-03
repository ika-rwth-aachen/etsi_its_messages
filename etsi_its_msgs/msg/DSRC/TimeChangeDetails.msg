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

# optional: True
TimeMark startTime

TimeMark minEndTime

# optional: True
TimeMark maxEndTime

# optional: True
TimeMark likelyTime

# optional: True
TimeIntervalConfidence confidence

# optional: True
TimeMark nextTime
