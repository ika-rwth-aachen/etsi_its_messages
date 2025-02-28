/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_H_
#define	_mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode {
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_unavailable	= 0,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_rockfalls	= 1,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_earthquakeDamage	= 2,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_sewerCollapse	= 3,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_subsidence	= 4,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_snowDrifts	= 5,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_stormDamage	= 6,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_burstPipe	= 7,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_volcanoEruption	= 8,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_fallingIce	= 9,
	mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_fire	= 10
} e_mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode;

/* mcm_ts_HazardousLocation-SurfaceConditionSubCauseCode */
typedef long	 mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode;
asn_struct_free_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_free;
asn_struct_print_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_print;
asn_constr_check_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_constraint;
per_type_decoder_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_decode_uper;
per_type_encoder_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_encode_uper;
per_type_decoder_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_decode_aper;
per_type_encoder_f mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_HazardousLocation_SurfaceConditionSubCauseCode_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
