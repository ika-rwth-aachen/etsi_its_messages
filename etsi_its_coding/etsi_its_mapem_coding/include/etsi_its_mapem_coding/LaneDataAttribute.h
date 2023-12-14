/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_coding/DeltaAngle.h"
#include "etsi_its_mapem_coding/RoadwayCrownAngle.h"
#include "etsi_its_mapem_coding/MergeDivergeNodeAngle.h"
#include "etsi_its_mapem_coding/SpeedLimitList.h"
#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/constr_CHOICE.h>
#ifndef	_LaneDataAttribute_H_
#define	_LaneDataAttribute_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LaneDataAttribute_PR {
	LaneDataAttribute_PR_NOTHING,	/* No components present */
	LaneDataAttribute_PR_pathEndPointAngle,
	LaneDataAttribute_PR_laneCrownPointCenter,
	LaneDataAttribute_PR_laneCrownPointLeft,
	LaneDataAttribute_PR_laneCrownPointRight,
	LaneDataAttribute_PR_laneAngle,
	LaneDataAttribute_PR_speedLimits,
	LaneDataAttribute_PR_regional
	/* Extensions may appear below */
	
} LaneDataAttribute_PR;

/* Forward declarations */
struct Reg_LaneDataAttribute;

/* LaneDataAttribute */
typedef struct LaneDataAttribute {
	LaneDataAttribute_PR present;
	union LaneDataAttribute_u {
		DeltaAngle_t	 pathEndPointAngle;
		RoadwayCrownAngle_t	 laneCrownPointCenter;
		RoadwayCrownAngle_t	 laneCrownPointLeft;
		RoadwayCrownAngle_t	 laneCrownPointRight;
		MergeDivergeNodeAngle_t	 laneAngle;
		SpeedLimitList_t	 speedLimits;
		struct LaneDataAttribute__regional {
			A_SEQUENCE_OF(struct Reg_LaneDataAttribute) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} regional;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LaneDataAttribute_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LaneDataAttribute;
extern asn_CHOICE_specifics_t asn_SPC_LaneDataAttribute_specs_1;
extern asn_TYPE_member_t asn_MBR_LaneDataAttribute_1[7];
extern asn_per_constraints_t asn_PER_type_LaneDataAttribute_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_mapem_coding/RegionalExtension.h"

#endif	/* _LaneDataAttribute_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
