/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_vam_ts_SafeDistanceIndicator_H_
#define	_vam_ts_SafeDistanceIndicator_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_vam_ts_coding/BOOLEAN.h>

#ifdef __cplusplus
extern "C" {
#endif

/* vam_ts_SafeDistanceIndicator */
typedef BOOLEAN_t	 vam_ts_SafeDistanceIndicator_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_SafeDistanceIndicator;
asn_struct_free_f vam_ts_SafeDistanceIndicator_free;
asn_struct_print_f vam_ts_SafeDistanceIndicator_print;
asn_constr_check_f vam_ts_SafeDistanceIndicator_constraint;
jer_type_encoder_f vam_ts_SafeDistanceIndicator_encode_jer;
per_type_decoder_f vam_ts_SafeDistanceIndicator_decode_uper;
per_type_encoder_f vam_ts_SafeDistanceIndicator_encode_uper;
per_type_decoder_f vam_ts_SafeDistanceIndicator_decode_aper;
per_type_encoder_f vam_ts_SafeDistanceIndicator_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _vam_ts_SafeDistanceIndicator_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>
