# ------------------------------------------------------------------------------
# SlowVehicleSubCauseCode ::= INTEGER {unavailable(0), maintenanceVehicle(1), vehiclesSlowingToLookAtAccident(2), abnormalLoad(3), abnormalWideLoad(4), convoy(5), snowplough(6), deicing(7), saltingVehicles(8)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 MAINTENANCE_VEHICLE = 1
uint8 VEHICLES_SLOWING_TO_LOOK_AT_ACCIDENT = 2
uint8 ABNORMAL_LOAD = 3
uint8 ABNORMAL_WIDE_LOAD = 4
uint8 CONVOY = 5
uint8 SNOWPLOUGH = 6
uint8 DEICING = 7
uint8 SALTING_VEHICLES = 8
uint8 value
