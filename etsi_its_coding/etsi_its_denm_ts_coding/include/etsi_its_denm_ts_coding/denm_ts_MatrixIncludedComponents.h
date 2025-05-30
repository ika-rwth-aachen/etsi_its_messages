/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_ts_MatrixIncludedComponents_H_
#define	_denm_ts_MatrixIncludedComponents_H_


#include <etsi_its_denm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_ts_MatrixIncludedComponents {
	denm_ts_MatrixIncludedComponents_xPosition	= 0,
	denm_ts_MatrixIncludedComponents_yPosition	= 1,
	denm_ts_MatrixIncludedComponents_zPosition	= 2,
	denm_ts_MatrixIncludedComponents_xVelocityOrVelocityMagnitude	= 3,
	denm_ts_MatrixIncludedComponents_yVelocityOrVelocityDirection	= 4,
	denm_ts_MatrixIncludedComponents_zSpeed	= 5,
	denm_ts_MatrixIncludedComponents_xAccelOrAccelMagnitude	= 6,
	denm_ts_MatrixIncludedComponents_yAccelOrAccelDirection	= 7,
	denm_ts_MatrixIncludedComponents_zAcceleration	= 8,
	denm_ts_MatrixIncludedComponents_zAngle	= 9,
	denm_ts_MatrixIncludedComponents_yAngle	= 10,
	denm_ts_MatrixIncludedComponents_xAngle	= 11,
	denm_ts_MatrixIncludedComponents_zAngularVelocity	= 12
} e_denm_ts_MatrixIncludedComponents;

/* denm_ts_MatrixIncludedComponents */
typedef BIT_STRING_t	 denm_ts_MatrixIncludedComponents_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_ts_MatrixIncludedComponents_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_ts_MatrixIncludedComponents;
asn_struct_free_f denm_ts_MatrixIncludedComponents_free;
asn_struct_print_f denm_ts_MatrixIncludedComponents_print;
asn_constr_check_f denm_ts_MatrixIncludedComponents_constraint;
jer_type_encoder_f denm_ts_MatrixIncludedComponents_encode_jer;
per_type_decoder_f denm_ts_MatrixIncludedComponents_decode_uper;
per_type_encoder_f denm_ts_MatrixIncludedComponents_encode_uper;
per_type_decoder_f denm_ts_MatrixIncludedComponents_decode_aper;
per_type_encoder_f denm_ts_MatrixIncludedComponents_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_ts_MatrixIncludedComponents_H_ */
#include <etsi_its_denm_ts_coding/asn_internal.h>
