/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_SequenceOfCartesianPosition3d_H_
#define	_mcm_ts_SequenceOfCartesianPosition3d_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/asn_SEQUENCE_OF.h>
#include <uulm_mcm_ts_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct mcm_ts_CartesianPosition3d;

/* mcm_ts_SequenceOfCartesianPosition3d */
typedef struct mcm_ts_SequenceOfCartesianPosition3d {
	A_SEQUENCE_OF(struct mcm_ts_CartesianPosition3d) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_SequenceOfCartesianPosition3d_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_SequenceOfCartesianPosition3d;
extern asn_SET_OF_specifics_t asn_SPC_mcm_ts_SequenceOfCartesianPosition3d_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_ts_SequenceOfCartesianPosition3d_1[1];
extern asn_per_constraints_t asn_PER_type_mcm_ts_SequenceOfCartesianPosition3d_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "uulm_mcm_ts_coding/mcm_ts_CartesianPosition3d.h"

#endif	/* _mcm_ts_SequenceOfCartesianPosition3d_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
