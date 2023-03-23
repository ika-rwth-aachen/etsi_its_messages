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

LowFrequencyContainer lowFrequencyContainer
bool lowFrequencyContainer_isPresent

SpecialVehicleContainer specialVehicleContainer
bool specialVehicleContainer_isPresent
