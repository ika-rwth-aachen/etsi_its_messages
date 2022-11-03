# ------------------------------------------------------------------------------
# TisTpgVDRM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  fillingStatus FillingStatus,
#  automationLevel TpgAutomation, 
#  pairingID	PairingID OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

FillingStatus fillingStatus

TpgAutomation automationLevel

# optional: True
PairingID pairingID
