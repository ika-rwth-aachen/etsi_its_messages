# ------------------------------------------------------------------------------
# LightBarSirenInUse ::= BIT STRING {
#     lightBarActivated (0),
#     sirenActivated (1)
# } (SIZE(2))
# ------------------------------------------------------------------------------

# size: [2]
uint8 INDEX_LIGHT_BAR_ACTIVATED = 0
uint8 INDEX_SIREN_ACTIVATED = 1
bool[2] value
