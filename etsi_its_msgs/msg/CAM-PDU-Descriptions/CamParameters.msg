# ------------------------------------------------------------------------------
# CamParameters ::= SEQUENCE {
# 	basicContainer BasicContainer,
# 	highFrequencyContainer HighFrequencyContainer,
# 	lowFrequencyContainer LowFrequencyContainer OPTIONAL,
# 	specialVehicleContainer SpecialVehicleContainer OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

BasicContainer basicContainer

HighFrequencyContainer highFrequencyContainer

# optional: True
LowFrequencyContainer lowFrequencyContainer

# optional: True
SpecialVehicleContainer specialVehicleContainer
