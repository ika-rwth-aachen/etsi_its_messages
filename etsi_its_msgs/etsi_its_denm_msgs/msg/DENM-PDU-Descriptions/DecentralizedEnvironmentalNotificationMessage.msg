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
bool situation_is_present

LocationContainer location
bool location_is_present

AlacarteContainer alacarte
bool alacarte_is_present

