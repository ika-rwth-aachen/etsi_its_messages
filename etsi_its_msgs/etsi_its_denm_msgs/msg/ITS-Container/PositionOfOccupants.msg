# ------------------------------------------------------------------------------
# PositionOfOccupants ::= BIT STRING {
#     row1LeftOccupied (0),
#     row1RightOccupied (1),
#     row1MidOccupied (2),
#     row1NotDetectable (3),
#     row1NotPresent (4),
#     row2LeftOccupied (5),
#     row2RightOccupied (6),
#     row2MidOccupied (7),
#     row2NotDetectable (8),
#     row2NotPresent (9),
#     row3LeftOccupied (10),
#     row3RightOccupied (11),
#     row3MidOccupied (12),
#     row3NotDetectable (13),
#     row3NotPresent (14),
#     row4LeftOccupied (15),
#     row4RightOccupied (16),
#     row4MidOccupied (17),
#     row4NotDetectable (18),
#     row4NotPresent (19)} (SIZE(20))
# ------------------------------------------------------------------------------

uint8[] value
uint8 bits_unused
uint8 SIZE_BITS = 20
uint8 BIT_INDEX_ROW_1_LEFT_OCCUPIED = 0
uint8 BIT_INDEX_ROW_1_RIGHT_OCCUPIED = 1
uint8 BIT_INDEX_ROW_1_MID_OCCUPIED = 2
uint8 BIT_INDEX_ROW_1_NOT_DETECTABLE = 3
uint8 BIT_INDEX_ROW_1_NOT_PRESENT = 4
uint8 BIT_INDEX_ROW_2_LEFT_OCCUPIED = 5
uint8 BIT_INDEX_ROW_2_RIGHT_OCCUPIED = 6
uint8 BIT_INDEX_ROW_2_MID_OCCUPIED = 7
uint8 BIT_INDEX_ROW_2_NOT_DETECTABLE = 8
uint8 BIT_INDEX_ROW_2_NOT_PRESENT = 9
uint8 BIT_INDEX_ROW_3_LEFT_OCCUPIED = 10
uint8 BIT_INDEX_ROW_3_RIGHT_OCCUPIED = 11
uint8 BIT_INDEX_ROW_3_MID_OCCUPIED = 12
uint8 BIT_INDEX_ROW_3_NOT_DETECTABLE = 13
uint8 BIT_INDEX_ROW_3_NOT_PRESENT = 14
uint8 BIT_INDEX_ROW_4_LEFT_OCCUPIED = 15
uint8 BIT_INDEX_ROW_4_RIGHT_OCCUPIED = 16
uint8 BIT_INDEX_ROW_4_MID_OCCUPIED = 17
uint8 BIT_INDEX_ROW_4_NOT_DETECTABLE = 18
uint8 BIT_INDEX_ROW_4_NOT_PRESENT = 19

