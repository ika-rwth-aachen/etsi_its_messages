# ------------------------------------------------------------------------------
# CamParameters ::= SEQUENCE {
# 	basicContainer BasicContainer,
# 	highFrequencyContainer HighFrequencyContainer,
# 	lowFrequencyContainer LowFrequencyContainer OPTIONAL,
# 	specialVehicleContainer SpecialVehicleContainer OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

BasicContainer basic_container

HighFrequencyContainer high_frequency_container

LowFrequencyContainer low_frequency_container
bool low_frequency_container_is_present

SpecialVehicleContainer special_vehicle_container
bool special_vehicle_container_is_present

