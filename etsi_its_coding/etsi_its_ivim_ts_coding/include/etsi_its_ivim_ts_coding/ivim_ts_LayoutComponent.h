/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_LayoutComponent_H_
#define	_ivim_ts_LayoutComponent_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_LayoutComponent__textScripting {
	ivim_ts_LayoutComponent__textScripting_horizontal	= 0,
	ivim_ts_LayoutComponent__textScripting_vertical	= 1
} e_ivim_ts_LayoutComponent__textScripting;

/* ivim_ts_LayoutComponent */
typedef struct ivim_ts_LayoutComponent {
	long	 layoutComponentId;
	long	 height;
	long	 width;
	long	 x;
	long	 y;
	long	 textScripting;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_LayoutComponent_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_LayoutComponent;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_LayoutComponent_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_LayoutComponent_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_LayoutComponent_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
