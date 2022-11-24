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
# optional: True
string processMethod

# size: [(1, 255)]
# optional: True
string processAgency

# size: [(1, 255)]
# optional: True
string lastCheckedDate

# size: [(1, 255)]
# optional: True
string geoidUsed
