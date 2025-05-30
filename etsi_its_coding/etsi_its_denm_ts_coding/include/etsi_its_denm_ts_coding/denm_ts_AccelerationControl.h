/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_AccelerationControl_H_
#define	_denm_ts_AccelerationControl_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_ts_AccelerationControl {
	denm_ts_AccelerationControl_brakePedalEngaged	= 0,
	denm_ts_AccelerationControl_gasPedalEngaged	= 1,
	denm_ts_AccelerationControl_emergencyBrakeEngaged	= 2,
	denm_ts_AccelerationControl_collisionWarningEngaged	= 3,
	denm_ts_AccelerationControl_accEngaged	= 4,
	denm_ts_AccelerationControl_cruiseControlEngaged	= 5,
	denm_ts_AccelerationControl_speedLimiterEngaged	= 6
} e_denm_ts_AccelerationControl;

/* denm_ts_AccelerationControl */
typedef BIT_STRING_t	 denm_ts_AccelerationControl_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_AccelerationControl;
asn_struct_free_f denm_ts_AccelerationControl_free;
asn_struct_print_f denm_ts_AccelerationControl_print;
asn_constr_check_f denm_ts_AccelerationControl_constraint;
jer_type_encoder_f denm_ts_AccelerationControl_encode_jer;
per_type_decoder_f denm_ts_AccelerationControl_decode_uper;
per_type_encoder_f denm_ts_AccelerationControl_encode_uper;
per_type_decoder_f denm_ts_AccelerationControl_decode_aper;
per_type_encoder_f denm_ts_AccelerationControl_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_AccelerationControl_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
