# ------------------------------------------------------------------------------
# HighFrequencyContainer ::= CHOICE {
# 	basicVehicleContainerHighFrequency BasicVehicleContainerHighFrequency,
# 	rsuContainerHighFrequency RSUContainerHighFrequency,
# 	...
# }
# ------------------------------------------------------------------------------

uint8 choice

BasicVehicleContainerHighFrequency basic_vehicle_container_high_frequency
uint8 CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY = 0

RSUContainerHighFrequency rsu_container_high_frequency
uint8 CHOICE_RSU_CONTAINER_HIGH_FREQUENCY = 1

