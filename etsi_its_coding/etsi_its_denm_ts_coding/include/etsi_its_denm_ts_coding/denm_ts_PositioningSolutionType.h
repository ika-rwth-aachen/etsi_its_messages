/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_PositioningSolutionType_H_
#define	_denm_ts_PositioningSolutionType_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_ts_PositioningSolutionType {
	denm_ts_PositioningSolutionType_noPositioningSolution	= 0,
	denm_ts_PositioningSolutionType_sGNSS	= 1,
	denm_ts_PositioningSolutionType_dGNSS	= 2,
	denm_ts_PositioningSolutionType_sGNSSplusDR	= 3,
	denm_ts_PositioningSolutionType_dGNSSplusDR	= 4,
	denm_ts_PositioningSolutionType_dR	= 5,
	/*
	 * Enumeration is extensible
	 */
	denm_ts_PositioningSolutionType_manuallyByOperator	= 6
} e_denm_ts_PositioningSolutionType;

/* denm_ts_PositioningSolutionType */
typedef long	 denm_ts_PositioningSolutionType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_ts_PositioningSolutionType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_PositioningSolutionType;
extern const asn_INTEGER_specifics_t asn_SPC_denm_ts_PositioningSolutionType_specs_1;
asn_struct_free_f denm_ts_PositioningSolutionType_free;
asn_struct_print_f denm_ts_PositioningSolutionType_print;
asn_constr_check_f denm_ts_PositioningSolutionType_constraint;
jer_type_encoder_f denm_ts_PositioningSolutionType_encode_jer;
per_type_decoder_f denm_ts_PositioningSolutionType_decode_uper;
per_type_encoder_f denm_ts_PositioningSolutionType_encode_uper;
per_type_decoder_f denm_ts_PositioningSolutionType_decode_aper;
per_type_encoder_f denm_ts_PositioningSolutionType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_PositioningSolutionType_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
