# ------------------------------------------------------------------------------
# SPATEM ::= SEQUENCE {
#     -- @details header
#     -- The DE _protocolVersion_ is used to select the appropriate protocol decoder at the receiving ITS-S.
#     --   It shall be set to 2.
#     -- The DE _messageID_ shall be spatem(4).
#     header  ItsPduHeader,
#     -- @details spat
#     -- The DE _spat_ contains the SPaT data as defined in ISO TS 19091.
#     spat    SPAT
# }
# ------------------------------------------------------------------------------

ItsPduHeader header

SPAT spat
