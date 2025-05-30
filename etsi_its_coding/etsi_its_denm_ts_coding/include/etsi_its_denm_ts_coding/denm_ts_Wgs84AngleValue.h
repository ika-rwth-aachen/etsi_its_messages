/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_Wgs84AngleValue_H_
#define	_denm_ts_Wgs84AngleValue_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_ts_Wgs84AngleValue {
	denm_ts_Wgs84AngleValue_wgs84North	= 0,
	denm_ts_Wgs84AngleValue_wgs84East	= 900,
	denm_ts_Wgs84AngleValue_wgs84South	= 1800,
	denm_ts_Wgs84AngleValue_wgs84West	= 2700,
	denm_ts_Wgs84AngleValue_doNotUse	= 3600,
	denm_ts_Wgs84AngleValue_unavailable	= 3601
} e_denm_ts_Wgs84AngleValue;

/* denm_ts_Wgs84AngleValue */
typedef long	 denm_ts_Wgs84AngleValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_ts_Wgs84AngleValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_Wgs84AngleValue;
asn_struct_free_f denm_ts_Wgs84AngleValue_free;
asn_struct_print_f denm_ts_Wgs84AngleValue_print;
asn_constr_check_f denm_ts_Wgs84AngleValue_constraint;
jer_type_encoder_f denm_ts_Wgs84AngleValue_encode_jer;
per_type_decoder_f denm_ts_Wgs84AngleValue_decode_uper;
per_type_encoder_f denm_ts_Wgs84AngleValue_encode_uper;
per_type_decoder_f denm_ts_Wgs84AngleValue_decode_aper;
per_type_encoder_f denm_ts_Wgs84AngleValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_Wgs84AngleValue_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
