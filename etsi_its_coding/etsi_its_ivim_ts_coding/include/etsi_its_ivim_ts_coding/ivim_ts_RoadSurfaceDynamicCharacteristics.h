/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_RoadSurfaceDynamicCharacteristics_H_
#define	_ivim_ts_RoadSurfaceDynamicCharacteristics_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_Condition.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_IVI_Temperature.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Depth.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_TreatmentType.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_RoadSurfaceDynamicCharacteristics */
typedef struct ivim_ts_RoadSurfaceDynamicCharacteristics {
	ivim_ts_Condition_t	 condition;
	ivim_ts_IVI_Temperature_t	 temperature;
	ivim_ts_Depth_t	 iceOrWaterDepth;
	ivim_ts_TreatmentType_t	 treatment;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_RoadSurfaceDynamicCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_RoadSurfaceDynamicCharacteristics;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_RoadSurfaceDynamicCharacteristics_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_RoadSurfaceDynamicCharacteristics_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_RoadSurfaceDynamicCharacteristics_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
