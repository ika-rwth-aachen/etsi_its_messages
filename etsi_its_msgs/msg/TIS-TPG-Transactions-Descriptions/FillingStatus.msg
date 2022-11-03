# ------------------------------------------------------------------------------
# FillingStatus ::= INTEGER {requestVehicleData(0), sendVehicleData(1), started(2), fillingProcessFailed (3), fillingProcessCompleted (4) } (0..7)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 7)]
uint8 REQUEST_VEHICLE_DATA = 0
uint8 SEND_VEHICLE_DATA = 1
uint8 STARTED = 2
uint8 FILLING_PROCESS_FAILED = 3
uint8 FILLING_PROCESS_COMPLETED = 4
uint8 value
