/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_vam_ts_VelocityComponentValue_H_
#define	_vam_ts_VelocityComponentValue_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_vam_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum vam_ts_VelocityComponentValue {
	vam_ts_VelocityComponentValue_negativeOutOfRange	= -16383,
	vam_ts_VelocityComponentValue_positiveOutOfRange	= 16382,
	vam_ts_VelocityComponentValue_unavailable	= 16383
} e_vam_ts_VelocityComponentValue;

/* vam_ts_VelocityComponentValue */
typedef long	 vam_ts_VelocityComponentValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_vam_ts_VelocityComponentValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_VelocityComponentValue;
asn_struct_free_f vam_ts_VelocityComponentValue_free;
asn_struct_print_f vam_ts_VelocityComponentValue_print;
asn_constr_check_f vam_ts_VelocityComponentValue_constraint;
jer_type_encoder_f vam_ts_VelocityComponentValue_encode_jer;
per_type_decoder_f vam_ts_VelocityComponentValue_decode_uper;
per_type_encoder_f vam_ts_VelocityComponentValue_encode_uper;
per_type_decoder_f vam_ts_VelocityComponentValue_decode_aper;
per_type_encoder_f vam_ts_VelocityComponentValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _vam_ts_VelocityComponentValue_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>
