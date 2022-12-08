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
bool lowFrequencyContainer_isPresent

# optional: True
SpecialVehicleContainer specialVehicleContainer
bool specialVehicleContainer_isPresent
