# ------------------------------------------------------------------------------
# TisTpgEOFM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  fillingStatus FillingStatus,
#  numberOfAppliedPressure NumberOfAppliedPressure,
#  appliedTyrePressures AppliedTyrePressures OPTIONAL,
#  pairingID	PairingID OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

FillingStatus fillingStatus

NumberOfAppliedPressure numberOfAppliedPressure

# optional: True
AppliedTyrePressures appliedTyrePressures

# optional: True
PairingID pairingID
