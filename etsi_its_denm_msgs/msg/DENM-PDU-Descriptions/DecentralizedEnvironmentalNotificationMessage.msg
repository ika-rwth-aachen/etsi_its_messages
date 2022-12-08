# ------------------------------------------------------------------------------
# DecentralizedEnvironmentalNotificationMessage ::= SEQUENCE {
# 	management ManagementContainer,
# 	situation SituationContainer OPTIONAL,
# 	location LocationContainer OPTIONAL,
# 	alacarte AlacarteContainer OPTIONAL
# }
# ------------------------------------------------------------------------------

ManagementContainer management

SituationContainer situation
bool situation_isPresent

LocationContainer location
bool location_isPresent

AlacarteContainer alacarte
bool alacarte_isPresent
