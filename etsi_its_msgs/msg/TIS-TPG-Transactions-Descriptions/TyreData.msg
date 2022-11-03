# ------------------------------------------------------------------------------
# TyreData ::= SEQUENCE
# ------------------------------------------------------------------------------

# optional: True
uint8 currentTyrePressure_choice

TyrePressure tyrePressureValue
uint8 CURRENT_TYRE_PRESSURE_CHOICE_TYRE_PRESSURE_VALUE = 0

uint8 CURRENT_TYRE_PRESSURE_CHOICE_UNAVAILABLE = 1

# optional: True
uint8 tyreSidewallInformation_choice

TyreSidewallInformation tyreSidewallInformationValue
uint8 TYRE_SIDEWALL_INFORMATION_CHOICE_TYRE_SIDEWALL_INFORMATION_VALUE = 0

uint8 TYRE_SIDEWALL_INFORMATION_CHOICE_UNAVAILABLE = 1

# optional: True
uint8 currentInsideAirTemperature_choice

TyreAirTemperature tyreAirTemperatureValue
uint8 CURRENT_INSIDE_AIR_TEMPERATURE_CHOICE_TYRE_AIR_TEMPERATURE_VALUE = 0

uint8 CURRENT_INSIDE_AIR_TEMPERATURE_CHOICE_UNAVAILABLE = 1

# optional: True
uint8 recommendedTyrePressure_choice

AxlePlacardPressure axlePlacardPressureValue
uint8 RECOMMENDED_TYRE_PRESSURE_CHOICE_AXLE_PLACARD_PRESSURE_VALUE = 0

uint8 RECOMMENDED_TYRE_PRESSURE_CHOICE_UNAVAILABLE = 1

# optional: True
uint8 tin_choice

TIN tinValue
uint8 TIN_CHOICE_TIN_VALUE = 0

uint8 TIN_CHOICE_UNAVAILABLE = 1

# optional: True
uint8 sensorState_choice

SensorState sensorStateValue
uint8 SENSOR_STATE_CHOICE_SENSOR_STATE_VALUE = 0

uint8 SENSOR_STATE_CHOICE_UNAVAILABLE = 1
