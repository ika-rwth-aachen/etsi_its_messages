/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cpm_ts_Path_H_
#define	_cpm_ts_Path_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_cpm_ts_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cpm_ts_PathPoint;

/* cpm_ts_Path */
typedef struct cpm_ts_Path {
	A_SEQUENCE_OF(struct cpm_ts_PathPoint) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cpm_ts_Path_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_Path;
extern asn_SET_OF_specifics_t asn_SPC_cpm_ts_Path_specs_1;
extern asn_TYPE_member_t asn_MBR_cpm_ts_Path_1[1];
extern asn_per_constraints_t asn_PER_type_cpm_ts_Path_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cpm_ts_coding/cpm_ts_PathPoint.h"

#endif	/* _cpm_ts_Path_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>