# ------------------------------------------------------------------------------
# EuVehicleCategoryCode ::= CHOICE {
#   euVehicleCategoryL	EuVehicleCategoryL,   -- conforms to EU 2002/24 and UNECE 1999
#   euVehicleCategoryM	EuVehicleCategoryM,   -- conforms to EU 2001/116 and UNECE 1999
#   euVehicleCategoryN	EuVehicleCategoryN,   -- conforms to EU 2001/116 and UNECE 1999
#   euVehicleCategoryO	EuVehicleCategoryO,   -- conforms to EU 2001/116 and UNECE 1999
#   euVehilcleCategoryT	NULL,    -- conforms to UNECE 1999
#   euVehilcleCategoryG	NULL     -- conforms to EU 2001/116 and UNECE 1999
# }
# ------------------------------------------------------------------------------

uint8 choice

EuVehicleCategoryL euVehicleCategoryL
uint8 CHOICE_EU_VEHICLE_CATEGORY_L = 0
EuVehicleCategoryM euVehicleCategoryM
uint8 CHOICE_EU_VEHICLE_CATEGORY_M = 1
EuVehicleCategoryN euVehicleCategoryN
uint8 CHOICE_EU_VEHICLE_CATEGORY_N = 2
EuVehicleCategoryO euVehicleCategoryO
uint8 CHOICE_EU_VEHICLE_CATEGORY_O = 3

uint8 CHOICE_EU_VEHILCLE_CATEGORY_T = 4

uint8 CHOICE_EU_VEHILCLE_CATEGORY_G = 5
