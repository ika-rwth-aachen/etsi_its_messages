# ------------------------------------------------------------------------------
# StationType ::= INTEGER {unknown(0), pedestrian(1), cyclist(2), moped(3), motorcycle(4), passengerCar(5), bus(6), 
# lightTruck(7), heavyTruck(8), trailer(9), specialVehicles(10), tram(11), roadSideUnit(15)} (0..255)
# ------------------------------------------------------------------------------

uint8 value
uint8 MIN = 0
uint8 MAX = 255
uint8 UNKNOWN = 0
uint8 PEDESTRIAN = 1
uint8 CYCLIST = 2
uint8 MOPED = 3
uint8 MOTORCYCLE = 4
uint8 PASSENGER_CAR = 5
uint8 BUS = 6
uint8 LIGHT_TRUCK = 7
uint8 HEAVY_TRUCK = 8
uint8 TRAILER = 9
uint8 SPECIAL_VEHICLES = 10
uint8 TRAM = 11
uint8 ROAD_SIDE_UNIT = 15

