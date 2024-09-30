/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/NativeInteger.h>
#ifndef	_SteeringWheelAngleValue_H_
#define	_SteeringWheelAngleValue_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SteeringWheelAngleValue {
	SteeringWheelAngleValue_straight	= 0,
	SteeringWheelAngleValue_onePointFiveDegreesToRight	= -1,
	SteeringWheelAngleValue_onePointFiveDegreesToLeft	= 1,
	SteeringWheelAngleValue_unavailable	= 512
} e_SteeringWheelAngleValue;

/* SteeringWheelAngleValue */
typedef long	 SteeringWheelAngleValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SteeringWheelAngleValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SteeringWheelAngleValue;
asn_struct_free_f SteeringWheelAngleValue_free;
asn_struct_print_f SteeringWheelAngleValue_print;
asn_constr_check_f SteeringWheelAngleValue_constraint;
ber_type_decoder_f SteeringWheelAngleValue_decode_ber;
der_type_encoder_f SteeringWheelAngleValue_encode_der;
xer_type_decoder_f SteeringWheelAngleValue_decode_xer;
xer_type_encoder_f SteeringWheelAngleValue_encode_xer;
jer_type_encoder_f SteeringWheelAngleValue_encode_jer;
oer_type_decoder_f SteeringWheelAngleValue_decode_oer;
oer_type_encoder_f SteeringWheelAngleValue_encode_oer;
per_type_decoder_f SteeringWheelAngleValue_decode_uper;
per_type_encoder_f SteeringWheelAngleValue_encode_uper;
per_type_decoder_f SteeringWheelAngleValue_decode_aper;
per_type_encoder_f SteeringWheelAngleValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SteeringWheelAngleValue_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
