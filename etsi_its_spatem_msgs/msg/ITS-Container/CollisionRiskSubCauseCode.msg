# ------------------------------------------------------------------------------
# CollisionRiskSubCauseCode ::= INTEGER {unavailable(0), longitudinalCollisionRisk(1), crossingCollisionRisk(2), lateralCollisionRisk(3), vulnerableRoadUser(4)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 LONGITUDINAL_COLLISION_RISK = 1
uint8 CROSSING_COLLISION_RISK = 2
uint8 LATERAL_COLLISION_RISK = 3
uint8 VULNERABLE_ROAD_USER = 4
uint8 value
