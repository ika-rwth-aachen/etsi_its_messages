# ------------------------------------------------------------------------------
# LaneAttributes-Barrier ::= BIT STRING {
#   median-RevocableLane     (0),
#   median                   (1),
#   whiteLineHashing         (2),
#   stripedLines             (3),
#   doubleStripedLines       (4),
#   trafficCones             (5),
#   constructionBarrier      (6),
#   trafficChannels          (7),
#   lowCurbs                 (8),
#   highCurbs                (9)
# } (SIZE (16))
# ------------------------------------------------------------------------------

# named-bits: [('median-RevocableLane', '0'), ('median', '1'), ('whiteLineHashing', '2'), ('stripedLines', '3'), ('doubleStripedLines', '4'), ('trafficCones', '5'), ('constructionBarrier', '6'), ('trafficChannels', '7'), ('lowCurbs', '8'), ('highCurbs', '9')]
# size: [16]
string value
