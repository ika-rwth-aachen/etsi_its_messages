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

DangerousGoodsBasic dangerous_goods_type

uint16 un_number
uint16 UN_NUMBER_MIN = 0
uint16 UN_NUMBER_MAX = 9999

bool elevated_temperature

bool tunnels_restricted

bool limited_quantity

string emergency_action_code
bool emergency_action_code_is_present
uint8 EMERGENCY_ACTION_CODE_MIN_SIZE = 1
uint8 EMERGENCY_ACTION_CODE_MAX_SIZE = 24

PhoneNumber phone_number
bool phone_number_is_present

string company_name
bool company_name_is_present
uint8 COMPANY_NAME_MIN_SIZE = 1
uint8 COMPANY_NAME_MAX_SIZE = 24

