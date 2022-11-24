# ------------------------------------------------------------------------------
# LaneAttributes-Parking ::= BIT STRING { 
#   parkingRevocableLane         (0),
#   parallelParkingInUse         (1),
#   headInParkingInUse           (2),
#   doNotParkZone                (3),
#   parkingForBusUse             (4),
#   parkingForTaxiUse            (5),
#   noPublicParkingUse           (6)
# } (SIZE (16))
# ------------------------------------------------------------------------------

# named-bits: [('parkingRevocableLane', '0'), ('parallelParkingInUse', '1'), ('headInParkingInUse', '2'), ('doNotParkZone', '3'), ('parkingForBusUse', '4'), ('parkingForTaxiUse', '5'), ('noPublicParkingUse', '6')]
# size: [16]
string value
