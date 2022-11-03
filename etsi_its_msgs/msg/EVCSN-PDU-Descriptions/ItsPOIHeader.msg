# ------------------------------------------------------------------------------
# ItsPOIHeader ::= SEQUENCE {
# 	poiType                 POIType, -- set to "EV charging station POI ID = 1"
#     timeStamp               TimestampIts,
# 	relayCapable            BOOLEAN
# }
# ------------------------------------------------------------------------------

POIType poiType

TimestampIts timeStamp

bool relayCapable
