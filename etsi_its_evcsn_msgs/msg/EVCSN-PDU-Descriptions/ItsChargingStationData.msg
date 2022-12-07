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
string utilityDistributorId
bool utilityDistributorId_isPresent

# size: [(1, 32)]
string providerID
bool providerID_isPresent

ReferencePosition chargingStationLocation

string address
bool address_isPresent

# size: [(1, 16)]
string phoneNumber
bool phoneNumber_isPresent

# size: [(1, 32)]
string accessibility

DigitalMap digitalMap
bool digitalMap_isPresent

string openingDaysHours

string pricing

string bookingContactInfo
bool bookingContactInfo_isPresent

string payment
bool payment_isPresent

ItsChargingSpots chargingSpotsAvailable
