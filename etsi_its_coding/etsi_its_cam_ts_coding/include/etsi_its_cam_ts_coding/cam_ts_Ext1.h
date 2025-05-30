/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_Ext1_H_
#define	_cam_ts_Ext1_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeInteger.h>
#include "etsi_its_cam_ts_coding/cam_ts_Ext2.h"
#include <etsi_its_cam_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_Ext1_PR {
	cam_ts_Ext1_PR_NOTHING,	/* No components present */
	cam_ts_Ext1_PR_content,
	cam_ts_Ext1_PR_extension
} cam_ts_Ext1_PR;

/* cam_ts_Ext1 */
typedef struct cam_ts_Ext1 {
	cam_ts_Ext1_PR present;
	union cam_ts_Ext1_u {
		long	 content;
		cam_ts_Ext2_t	 extension;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_ts_Ext1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_Ext1;
extern asn_CHOICE_specifics_t asn_SPC_cam_ts_Ext1_specs_1;
extern asn_TYPE_member_t asn_MBR_cam_ts_Ext1_1[2];
extern asn_per_constraints_t asn_PER_type_cam_ts_Ext1_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_Ext1_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
