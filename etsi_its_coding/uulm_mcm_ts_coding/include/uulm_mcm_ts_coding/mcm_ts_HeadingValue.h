/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_HeadingValue_H_
#define	_mcm_ts_HeadingValue_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_HeadingValue {
	mcm_ts_HeadingValue_wgs84North	= 0,
	mcm_ts_HeadingValue_wgs84East	= 900,
	mcm_ts_HeadingValue_wgs84South	= 1800,
	mcm_ts_HeadingValue_wgs84West	= 2700,
	mcm_ts_HeadingValue_doNotUse	= 3600,
	mcm_ts_HeadingValue_unavailable	= 3601
} e_mcm_ts_HeadingValue;

/* mcm_ts_HeadingValue */
typedef long	 mcm_ts_HeadingValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_ts_HeadingValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_HeadingValue;
asn_struct_free_f mcm_ts_HeadingValue_free;
asn_struct_print_f mcm_ts_HeadingValue_print;
asn_constr_check_f mcm_ts_HeadingValue_constraint;
per_type_decoder_f mcm_ts_HeadingValue_decode_uper;
per_type_encoder_f mcm_ts_HeadingValue_encode_uper;
per_type_decoder_f mcm_ts_HeadingValue_decode_aper;
per_type_encoder_f mcm_ts_HeadingValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_HeadingValue_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
