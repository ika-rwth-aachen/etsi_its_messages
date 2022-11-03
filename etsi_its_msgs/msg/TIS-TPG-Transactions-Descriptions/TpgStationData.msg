# ------------------------------------------------------------------------------
# TpgStationData ::= SEQUENCE {
#  tpgStationID StationID,
#  tpgAutomationLevel TpgAutomation, 
#  tpgNumber TpgNumber,
#  tpgProvider TpgProvider,
#  tpgLocation ReferencePosition,
#  accessibility Accessibility,
#  address UTF8String(SIZE(1..128))  OPTIONAL,
#  phoneNumber PhoneNumber OPTIONAL,
#  digitalMap DigitalMap OPTIONAL,
#  openingDaysHours OpeningDaysHours OPTIONAL,
#  bookingInfo BookingInfo OPTIONAL,
#  availableTpgNumber AvailableTpgNumber OPTIONAL,
#  cancellationCondition CancellationCondition OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

StationID tpgStationID

TpgAutomation tpgAutomationLevel

TpgNumber tpgNumber

TpgProvider tpgProvider

ReferencePosition tpgLocation

Accessibility accessibility

# size: [(1, 128)]
# optional: True
string address

# optional: True
PhoneNumber phoneNumber

# optional: True
DigitalMap digitalMap

# optional: True
OpeningDaysHours openingDaysHours

# optional: True
BookingInfo bookingInfo

# optional: True
AvailableTpgNumber availableTpgNumber

# optional: True
CancellationCondition cancellationCondition
