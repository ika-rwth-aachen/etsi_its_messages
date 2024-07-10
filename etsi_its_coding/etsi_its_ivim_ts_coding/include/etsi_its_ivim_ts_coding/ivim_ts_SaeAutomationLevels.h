/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_SaeAutomationLevels_H_
#define	_ivim_ts_SaeAutomationLevels_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_SaeAutomationLevel.h"
#include <etsi_its_ivim_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_SaeAutomationLevels */
typedef struct ivim_ts_SaeAutomationLevels {
	A_SEQUENCE_OF(ivim_ts_SaeAutomationLevel_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_SaeAutomationLevels_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_SaeAutomationLevels;
extern asn_SET_OF_specifics_t asn_SPC_ivim_ts_SaeAutomationLevels_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_SaeAutomationLevels_1[1];
extern asn_per_constraints_t asn_PER_type_ivim_ts_SaeAutomationLevels_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_SaeAutomationLevels_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
