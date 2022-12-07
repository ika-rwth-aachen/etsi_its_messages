# ------------------------------------------------------------------------------
# DangerousGoodsExtended ::= SEQUENCE {
#     dangerousGoodsType DangerousGoodsBasic,
#     unNumber INTEGER (0..9999),
#     elevatedTemperature BOOLEAN,
#     tunnelsRestricted BOOLEAN,
#     limitedQuantity BOOLEAN,
#     emergencyActionCode IA5String (SIZE (1..24)) OPTIONAL,
#     phoneNumber PhoneNumber OPTIONAL,
#     companyName UTF8String (SIZE (1..24)) OPTIONAL,
#         ...
# }
# ------------------------------------------------------------------------------

DangerousGoodsBasic dangerousGoodsType

# restricted-to: [(0, 9999)]
uint16 unNumber

bool elevatedTemperature

bool tunnelsRestricted

bool limitedQuantity

# size: [(1, 24)]
string emergencyActionCode
bool emergencyActionCode_isPresent

PhoneNumber phoneNumber
bool phoneNumber_isPresent

# size: [(1, 24)]
string companyName
bool companyName_isPresent
