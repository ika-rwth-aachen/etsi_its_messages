/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_TractorCharacteristics_H_
#define	_ivim_ts_TractorCharacteristics_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivim_ts_VehicleCharacteristicsFixValuesList;
struct ivim_ts_VehicleCharacteristicsRangesList;

/* ivim_ts_TractorCharacteristics */
typedef struct ivim_ts_TractorCharacteristics {
	struct ivim_ts_VehicleCharacteristicsFixValuesList	*equalTo;	/* OPTIONAL */
	struct ivim_ts_VehicleCharacteristicsFixValuesList	*notEqualTo;	/* OPTIONAL */
	struct ivim_ts_VehicleCharacteristicsRangesList	*ranges;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_TractorCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_TractorCharacteristics;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_TractorCharacteristics_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_TractorCharacteristics_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_VehicleCharacteristicsFixValuesList.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_VehicleCharacteristicsRangesList.h"

#endif	/* _ivim_ts_TractorCharacteristics_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
