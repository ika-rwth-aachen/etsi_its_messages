# ------------------------------------------------------------------------------
# LightBarSirenInUse ::= BIT STRING {
#     lightBarActivated (0),
#     sirenActivated (1)
# } (SIZE(2))
# ------------------------------------------------------------------------------

uint8[] value
uint8 bits_unused
uint8 SIZE_BITS = 2
uint8 BIT_INDEX_LIGHT_BAR_ACTIVATED = 0
uint8 BIT_INDEX_SIREN_ACTIVATED = 1

