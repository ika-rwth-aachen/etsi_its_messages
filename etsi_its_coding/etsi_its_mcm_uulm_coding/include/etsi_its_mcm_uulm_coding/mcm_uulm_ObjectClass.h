/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_ObjectClass_H_
#define	_mcm_uulm_ObjectClass_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_mcm_uulm_coding/mcm_uulm_TrafficParticipantType.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_VruProfileAndSubprofile.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_VruClusterInformation.h"
#include "etsi_its_mcm_uulm_coding/mcm_uulm_OtherSubClass.h"
#include <etsi_its_mcm_uulm_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_ObjectClass_PR {
	mcm_uulm_ObjectClass_PR_NOTHING,	/* No components present */
	mcm_uulm_ObjectClass_PR_vehicleSubClass,
	mcm_uulm_ObjectClass_PR_vruSubClass,
	mcm_uulm_ObjectClass_PR_groupSubClass,
	mcm_uulm_ObjectClass_PR_otherSubClass
	/* Extensions may appear below */
	
} mcm_uulm_ObjectClass_PR;

/* mcm_uulm_ObjectClass */
typedef struct mcm_uulm_ObjectClass {
	mcm_uulm_ObjectClass_PR present;
	union mcm_uulm_ObjectClass_u {
		mcm_uulm_TrafficParticipantType_t	 vehicleSubClass;
		mcm_uulm_VruProfileAndSubprofile_t	 vruSubClass;
		mcm_uulm_VruClusterInformation_t	 groupSubClass;
		mcm_uulm_OtherSubClass_t	 otherSubClass;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} mcm_uulm_ObjectClass_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_ObjectClass;
extern asn_CHOICE_specifics_t asn_SPC_mcm_uulm_ObjectClass_specs_1;
extern asn_TYPE_member_t asn_MBR_mcm_uulm_ObjectClass_1[4];
extern asn_per_constraints_t asn_PER_type_mcm_uulm_ObjectClass_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_ObjectClass_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
