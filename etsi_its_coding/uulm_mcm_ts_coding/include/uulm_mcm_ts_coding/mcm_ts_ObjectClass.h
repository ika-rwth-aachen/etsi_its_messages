/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_ObjectClass_H_
#define	_mcm_ts_ObjectClass_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "uulm_mcm_ts_coding/mcm_ts_TrafficParticipantType.h"
#include "uulm_mcm_ts_coding/mcm_ts_VruProfileAndSubprofile.h"
#include "uulm_mcm_ts_coding/mcm_ts_VruClusterInformation.h"
#include "uulm_mcm_ts_coding/mcm_ts_OtherSubClass.h"
#include <uulm_mcm_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_ObjectClass_PR {
	mcm_ts_ObjectClass_PR_NOTHING,	/* No components present */
	mcm_ts_ObjectClass_PR_vehicleSubClass,
	mcm_ts_ObjectClass_PR_vruSubClass,
	mcm_ts_ObjectClass_PR_groupSubClass,
	mcm_ts_ObjectClass_PR_otherSubClass
	/* Extensions may appear below */
	
} mcm_ts_ObjectClass_PR;

/* mcm_ts_ObjectClass */
typedef struct mcm_ts_ObjectClass {
	mcm_ts_ObjectClass_PR present;
	union mcm_ts_ObjectClass_u {
		mcm_ts_TrafficParticipantType_t	 vehicleSubClass;
		mcm_ts_VruProfileAndSubprofile_t	 vruSubClass;
		mcm_ts_VruClusterInformation_t	 groupSubClass;
		mcm_ts_OtherSubClass_t	 otherSubClass;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_ts_ObjectClass_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_ObjectClass;
extern asn_CHOICE_specifics_t asn_SPC_mcm_ts_ObjectClass_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_ts_ObjectClass_1[4];
extern asn_per_constraints_t asn_PER_type_mcm_ts_ObjectClass_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_ObjectClass_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
