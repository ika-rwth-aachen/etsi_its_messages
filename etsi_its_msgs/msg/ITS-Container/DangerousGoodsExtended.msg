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
# optional: True
string emergencyActionCode

# optional: True
PhoneNumber phoneNumber

# size: [(1, 24)]
# optional: True
string companyName
