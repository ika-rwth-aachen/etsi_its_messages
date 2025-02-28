/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_Velocity3dWithConfidence_H_
#define	_mcm_ts_Velocity3dWithConfidence_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "uulm_mcm_ts_coding/mcm_ts_VelocityPolarWithZ.h"
#include "uulm_mcm_ts_coding/mcm_ts_VelocityCartesian.h"
#include <uulm_mcm_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_Velocity3dWithConfidence_PR {
	mcm_ts_Velocity3dWithConfidence_PR_NOTHING,	/* No components present */
	mcm_ts_Velocity3dWithConfidence_PR_polarVelocity,
	mcm_ts_Velocity3dWithConfidence_PR_cartesianVelocity
} mcm_ts_Velocity3dWithConfidence_PR;

/* mcm_ts_Velocity3dWithConfidence */
typedef struct mcm_ts_Velocity3dWithConfidence {
	mcm_ts_Velocity3dWithConfidence_PR present;
	union mcm_ts_Velocity3dWithConfidence_u {
		mcm_ts_VelocityPolarWithZ_t	 polarVelocity;
		mcm_ts_VelocityCartesian_t	 cartesianVelocity;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_Velocity3dWithConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_Velocity3dWithConfidence;
extern asn_CHOICE_specifics_t asn_SPC_mcm_ts_Velocity3dWithConfidence_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_ts_Velocity3dWithConfidence_1[2];
extern asn_per_constraints_t asn_PER_type_mcm_ts_Velocity3dWithConfidence_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_Velocity3dWithConfidence_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
