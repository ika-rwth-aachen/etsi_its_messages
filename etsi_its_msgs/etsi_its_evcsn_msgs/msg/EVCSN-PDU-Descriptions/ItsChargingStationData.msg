# ------------------------------------------------------------------------------
# ItsChargingStationData ::= SEQUENCE {
# 	chargingStationID     	StationID,
# 	utilityDistributorId  	UTF8String (SIZE(1..32))	OPTIONAL,
# 	providerID            	UTF8String (SIZE(1..32))	OPTIONAL,
# 	chargingStationLocation	ReferencePosition,
# 	address               	UTF8String 	        	OPTIONAL,
# 	phoneNumber           	NumericString (SIZE(1..16))    	OPTIONAL,
# 	accessibility         	UTF8String (SIZE(1..32)),
# 	digitalMap            	DigitalMap           		OPTIONAL,
# 	openingDaysHours      	UTF8String,
# 	pricing               	UTF8String,
# 	bookingContactInfo    	UTF8String          		OPTIONAL,
# 	payment               	UTF8String          		OPTIONAL,
# 	chargingSpotsAvailable  ItsChargingSpots,
# 	...
# }
# ------------------------------------------------------------------------------

StationID chargingStationID

# size: [(1, 32)]
# optional: True
string utilityDistributorId

# size: [(1, 32)]
# optional: True
string providerID

ReferencePosition chargingStationLocation

# optional: True
string address

# size: [(1, 16)]
# optional: True
string phoneNumber

# size: [(1, 32)]
string accessibility

# optional: True
DigitalMap digitalMap

string openingDaysHours

string pricing

# optional: True
string bookingContactInfo

# optional: True
string payment

ItsChargingSpots chargingSpotsAvailable
