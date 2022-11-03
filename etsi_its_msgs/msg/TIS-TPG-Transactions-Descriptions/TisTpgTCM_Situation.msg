# ------------------------------------------------------------------------------
# TisTpgTCM-Situation::= SEQUENCE {
#  pairingID PairingID OPTIONAL,
#  reservationTimeLimit TimestampIts,
#  cancellationCondition CancellationCondition OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

# optional: True
PairingID pairingID

TimestampIts reservationTimeLimit

# optional: True
CancellationCondition cancellationCondition
