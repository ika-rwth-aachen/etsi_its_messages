# ------------------------------------------------------------------------------
# SpecialVehicleContainer ::= CHOICE {
# 	publicTransportContainer PublicTransportContainer,
# 	specialTransportContainer SpecialTransportContainer,
# 	dangerousGoodsContainer DangerousGoodsContainer,
# 	roadWorksContainerBasic RoadWorksContainerBasic,
# 	rescueContainer RescueContainer,
# 	emergencyContainer EmergencyContainer,
# 	safetyCarContainer SafetyCarContainer,
# 	...
# }
# ------------------------------------------------------------------------------

uint8 choice

PublicTransportContainer public_transport_container
uint8 CHOICE_PUBLIC_TRANSPORT_CONTAINER = 0

SpecialTransportContainer special_transport_container
uint8 CHOICE_SPECIAL_TRANSPORT_CONTAINER = 1

DangerousGoodsContainer dangerous_goods_container
uint8 CHOICE_DANGEROUS_GOODS_CONTAINER = 2

RoadWorksContainerBasic road_works_container_basic
uint8 CHOICE_ROAD_WORKS_CONTAINER_BASIC = 3

RescueContainer rescue_container
uint8 CHOICE_RESCUE_CONTAINER = 4

EmergencyContainer emergency_container
uint8 CHOICE_EMERGENCY_CONTAINER = 5

SafetyCarContainer safety_car_container
uint8 CHOICE_SAFETY_CAR_CONTAINER = 6

