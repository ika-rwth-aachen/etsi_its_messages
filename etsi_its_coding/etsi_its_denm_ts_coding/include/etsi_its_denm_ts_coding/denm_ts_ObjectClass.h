/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_ObjectClass_H_
#define	_denm_ts_ObjectClass_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_denm_ts_coding/denm_ts_TrafficParticipantType.h"
#include "etsi_its_denm_ts_coding/denm_ts_VruProfileAndSubprofile.h"
#include "etsi_its_denm_ts_coding/denm_ts_VruClusterInformation.h"
#include "etsi_its_denm_ts_coding/denm_ts_OtherSubClass.h"
#include <etsi_its_denm_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_ts_ObjectClass_PR {
	denm_ts_ObjectClass_PR_NOTHING,	/* No components present */
	denm_ts_ObjectClass_PR_vehicleSubClass,
	denm_ts_ObjectClass_PR_vruSubClass,
	denm_ts_ObjectClass_PR_groupSubClass,
	denm_ts_ObjectClass_PR_otherSubClass
	/* Extensions may appear below */
	
} denm_ts_ObjectClass_PR;

/* denm_ts_ObjectClass */
typedef struct denm_ts_ObjectClass {
	denm_ts_ObjectClass_PR present;
	union denm_ts_ObjectClass_u {
		denm_ts_TrafficParticipantType_t	 vehicleSubClass;
		denm_ts_VruProfileAndSubprofile_t	 vruSubClass;
		denm_ts_VruClusterInformation_t	 groupSubClass;
		denm_ts_OtherSubClass_t	 otherSubClass;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_ts_ObjectClass_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_ObjectClass;
extern asn_CHOICE_specifics_t asn_SPC_denm_ts_ObjectClass_specs_1;
extern asn_TYPE_member_t asn_MBR_denm_ts_ObjectClass_1[4];
extern asn_per_constraints_t asn_PER_type_denm_ts_ObjectClass_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_ObjectClass_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
