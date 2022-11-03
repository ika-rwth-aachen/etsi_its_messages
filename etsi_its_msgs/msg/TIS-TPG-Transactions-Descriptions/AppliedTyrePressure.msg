# ------------------------------------------------------------------------------
# AppliedTyrePressure ::=  CHOICE {
#  tyrePressureValue TyrePressure,
#  unavailable NULL
# }
# ------------------------------------------------------------------------------

uint8 choice

TyrePressure tyrePressureValue
uint8 CHOICE_TYRE_PRESSURE_VALUE = 0

uint8 CHOICE_UNAVAILABLE = 1
