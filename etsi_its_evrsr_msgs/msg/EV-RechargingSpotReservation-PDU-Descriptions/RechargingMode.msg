# ------------------------------------------------------------------------------
# RechargingMode ::=  INTEGER { mode1(0), mode2(1), mode3(2), mode4(3), quickDrop(8), inductiveChargingWhileStationary(12), inductiveChargingWhileDriving(14) } (0..15)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 15)]
uint8 MODE_1 = 0
uint8 MODE_2 = 1
uint8 MODE_3 = 2
uint8 MODE_4 = 3
uint8 QUICK_DROP = 8
uint8 INDUCTIVE_CHARGING_WHILE_STATIONARY = 12
uint8 INDUCTIVE_CHARGING_WHILE_DRIVING = 14
uint8 value
