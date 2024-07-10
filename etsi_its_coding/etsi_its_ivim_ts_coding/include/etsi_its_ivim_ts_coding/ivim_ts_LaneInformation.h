/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_LaneInformation_H_
#define	_ivim_ts_LaneInformation_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_LanePosition.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Direction.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LaneType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LaneStatus.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_IviLaneWidth.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivim_ts_InternationalSign_applicablePeriod;
struct ivim_ts_CompleteVehicleCharacteristics;
struct ivim_ts_ZoneIds;
struct ivim_ts_LaneCharacteristics;
struct ivim_ts_RoadSurfaceStaticCharacteristics;
struct ivim_ts_RoadSurfaceDynamicCharacteristics;

/* ivim_ts_LaneInformation */
typedef struct ivim_ts_LaneInformation {
	ivim_ts_LanePosition_t	 laneNumber;
	ivim_ts_Direction_t	 direction;
	struct ivim_ts_InternationalSign_applicablePeriod	*validity;	/* OPTIONAL */
	ivim_ts_LaneType_t	 laneType;
	struct ivim_ts_CompleteVehicleCharacteristics	*laneTypeQualifier;	/* OPTIONAL */
	ivim_ts_LaneStatus_t	 laneStatus;
	ivim_ts_IviLaneWidth_t	*laneWidth;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct ivim_ts_LaneInformation__ext1 {
		struct ivim_ts_ZoneIds	*detectionZoneIds;	/* OPTIONAL */
		struct ivim_ts_ZoneIds	*relevanceZoneIds;	/* OPTIONAL */
		struct ivim_ts_LaneCharacteristics	*laneCharacteristics;	/* OPTIONAL */
		struct ivim_ts_RoadSurfaceStaticCharacteristics	*laneSurfaceStaticCharacteristics;	/* OPTIONAL */
		struct ivim_ts_RoadSurfaceDynamicCharacteristics	*laneSurfaceDynamicCharacteristics;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_LaneInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_LaneInformation;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_LaneInformation_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_LaneInformation_1[8];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_InternationalSign-applicablePeriod.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_CompleteVehicleCharacteristics.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ZoneIds.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LaneCharacteristics.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadSurfaceStaticCharacteristics.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadSurfaceDynamicCharacteristics.h"

#endif	/* _ivim_ts_LaneInformation_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
