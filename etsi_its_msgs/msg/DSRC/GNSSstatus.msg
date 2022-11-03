# ------------------------------------------------------------------------------
# GNSSstatus ::= BIT STRING {
#    unavailable               (0),
#    isHealthy                 (1),
#    isMonitored               (2),
#    baseStationType           (3),
#    aPDOPofUnder5             (4),
#    inViewOfUnder5            (5),
#    localCorrectionsPresent   (6),
#    networkCorrectionsPresent (7)
#    } (SIZE(8))
# ------------------------------------------------------------------------------

# named-bits: [('unavailable', '0'), ('isHealthy', '1'), ('isMonitored', '2'), ('baseStationType', '3'), ('aPDOPofUnder5', '4'), ('inViewOfUnder5', '5'), ('localCorrectionsPresent', '6'), ('networkCorrectionsPresent', '7')]
# size: [8]
string value
