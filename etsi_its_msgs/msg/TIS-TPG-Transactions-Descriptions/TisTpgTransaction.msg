# ------------------------------------------------------------------------------
# TisTpgTransaction ::= CHOICE {
# 	drm	TisTpgDRM,
# 	snm	TisTpgSNM,
# 	trm	TisTpgTRM,
# 	tcm	TisTpgTCM,
# 	vdrm TisTpgVDRM,
# 	vdpm TisTpgVDPM,
# 	eofm TisTpgEOFM
# }
# ------------------------------------------------------------------------------

uint8 choice

TisTpgDRM drm
uint8 CHOICE_DRM = 0
TisTpgSNM snm
uint8 CHOICE_SNM = 1
TisTpgTRM trm
uint8 CHOICE_TRM = 2
TisTpgTCM tcm
uint8 CHOICE_TCM = 3
TisTpgVDRM vdrm
uint8 CHOICE_VDRM = 4
TisTpgVDPM vdpm
uint8 CHOICE_VDPM = 5
TisTpgEOFM eofm
uint8 CHOICE_EOFM = 6
