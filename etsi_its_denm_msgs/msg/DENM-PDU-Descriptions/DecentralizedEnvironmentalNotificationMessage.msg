# ------------------------------------------------------------------------------
# DecentralizedEnvironmentalNotificationMessage ::= SEQUENCE {
# 	management ManagementContainer,
# 	situation SituationContainer OPTIONAL,
# 	location LocationContainer OPTIONAL,
# 	alacarte AlacarteContainer OPTIONAL
# }
# ------------------------------------------------------------------------------

ManagementContainer management

# optional: True
SituationContainer situation

# optional: True
LocationContainer location

# optional: True
AlacarteContainer alacarte
