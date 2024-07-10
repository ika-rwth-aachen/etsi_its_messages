/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_GicPart_H_
#define	_ivim_ts_GicPart_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_Direction.h"
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include "etsi_its_ivim_ts_coding/ivim_ts_IviType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_IviPurpose.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LaneStatus.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_IVI_DriverCharacteristics.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadSignCodes.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivim_ts_ZoneIds;
struct ivim_ts_VarLengthNumber;
struct ivim_ts_LanePositions;
struct ivim_ts_VehicleCharacteristicsList;
struct ivim_ts_ConstraintTextLines1;

/* ivim_ts_GicPart */
typedef struct ivim_ts_GicPart {
	struct ivim_ts_ZoneIds	*detectionZoneIds;	/* OPTIONAL */
	struct ivim_ts_VarLengthNumber	*its_Rrid;	/* OPTIONAL */
	struct ivim_ts_ZoneIds	*relevanceZoneIds;	/* OPTIONAL */
	ivim_ts_Direction_t	*direction;	/* OPTIONAL */
	struct ivim_ts_ZoneIds	*driverAwarenessZoneIds;	/* OPTIONAL */
	long	*minimumAwarenessTime;	/* OPTIONAL */
	struct ivim_ts_LanePositions	*applicableLanes;	/* OPTIONAL */
	ivim_ts_IviType_t	 iviType;
	ivim_ts_IviPurpose_t	*iviPurpose;	/* OPTIONAL */
	ivim_ts_LaneStatus_t	*laneStatus;	/* OPTIONAL */
	struct ivim_ts_VehicleCharacteristicsList	*vehicleCharacteristics;	/* OPTIONAL */
	ivim_ts_IVI_DriverCharacteristics_t	*driverCharacteristics;	/* OPTIONAL */
	long	*layoutId;	/* OPTIONAL */
	long	*preStoredlayoutId;	/* OPTIONAL */
	ivim_ts_RoadSignCodes_t	 roadSignCodes;
	struct ivim_ts_ConstraintTextLines1	*extraText;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_GicPart_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_GicPart;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_GicPart_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_GicPart_1[16];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_ZoneIds.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_VarLengthNumber.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LanePositions.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_VehicleCharacteristicsList.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ConstraintTextLines1.h"

#endif	/* _ivim_ts_GicPart_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
