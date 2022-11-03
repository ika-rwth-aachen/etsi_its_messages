# ------------------------------------------------------------------------------
# VehicleBreakdownSubCauseCode ::= INTEGER {unavailable(0), lackOfFuel (1), lackOfBatteryPower (2), engineProblem(3), transmissionProblem(4), engineCoolingProblem(5), brakingSystemProblem(6), steeringProblem(7), tyrePuncture(8), tyrePressureProblem(9)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 LACK_OF_FUEL = 1
uint8 LACK_OF_BATTERY_POWER = 2
uint8 ENGINE_PROBLEM = 3
uint8 TRANSMISSION_PROBLEM = 4
uint8 ENGINE_COOLING_PROBLEM = 5
uint8 BRAKING_SYSTEM_PROBLEM = 6
uint8 STEERING_PROBLEM = 7
uint8 TYRE_PUNCTURE = 8
uint8 TYRE_PRESSURE_PROBLEM = 9
uint8 value
