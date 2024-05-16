## SEQUENCE DangerousGoodsExtended .extensible

DangerousGoodsBasic dangerous_goods_type


uint16 un_number
uint16 UN_NUMBER_MIN = 0
uint16 UN_NUMBER_MAX = 9999

bool elevated_temperature


bool tunnels_restricted


bool limited_quantity


bool emergency_action_code_is_present
string emergency_action_code
uint8 EMERGENCY_ACTION_CODE_LENGTH_MIN = 1
uint8 EMERGENCY_ACTION_CODE_LENGTH_MAX = 24

bool phone_number_is_present
PhoneNumber phone_number


bool company_name_is_present
string company_name
uint8 COMPANY_NAME_LENGTH_MIN = 1
uint8 COMPANY_NAME_LENGTH_MAX = 24





