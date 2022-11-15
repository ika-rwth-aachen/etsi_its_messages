# ------------------------------------------------------------------------------
# MAPEM ::= SEQUENCE {
#     -- @details header
#     -- The DE _protocolVersion_ is used to select the appropriate protocol decoder at the receiving ITS-S.
#     --   It shall be set to 2.
#     -- The DE _messageID_ shall be mapem(5).
#     header  ItsPduHeader,
#     -- @details map
#     -- The DE _map_ contains the MAP data as defined in ISO TS 19091.
#     map	    MapData
# }
# ------------------------------------------------------------------------------

ItsPduHeader header

MapData map
