# ------------------------------------------------------------------------------
# DataParameters ::= SEQUENCE {
#   processMethod     IA5String(SIZE(1..255)) OPTIONAL, 
#   processAgency     IA5String(SIZE(1..255)) OPTIONAL, 
#   lastCheckedDate   IA5String(SIZE(1..255)) OPTIONAL, 
#   geoidUsed         IA5String(SIZE(1..255)) OPTIONAL, 
#   ...  
# }
# ------------------------------------------------------------------------------

# size: [(1, 255)]
string processMethod
bool processMethod_isPresent

# size: [(1, 255)]
string processAgency
bool processAgency_isPresent

# size: [(1, 255)]
string lastCheckedDate
bool lastCheckedDate_isPresent

# size: [(1, 255)]
string geoidUsed
bool geoidUsed_isPresent
