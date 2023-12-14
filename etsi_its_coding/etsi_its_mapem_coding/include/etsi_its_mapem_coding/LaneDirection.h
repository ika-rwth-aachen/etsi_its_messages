/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/BIT_STRING.h>
#ifndef	_LaneDirection_H_
#define	_LaneDirection_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LaneDirection {
	LaneDirection_ingressPath	= 0,
	LaneDirection_egressPath	= 1
} e_LaneDirection;

/* LaneDirection */
typedef BIT_STRING_t	 LaneDirection_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_LaneDirection_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_LaneDirection;
asn_struct_free_f LaneDirection_free;
asn_struct_print_f LaneDirection_print;
asn_constr_check_f LaneDirection_constraint;
ber_type_decoder_f LaneDirection_decode_ber;
der_type_encoder_f LaneDirection_encode_der;
xer_type_decoder_f LaneDirection_decode_xer;
xer_type_encoder_f LaneDirection_encode_xer;
jer_type_encoder_f LaneDirection_encode_jer;
oer_type_decoder_f LaneDirection_decode_oer;
oer_type_encoder_f LaneDirection_encode_oer;
per_type_decoder_f LaneDirection_decode_uper;
per_type_encoder_f LaneDirection_encode_uper;
per_type_decoder_f LaneDirection_decode_aper;
per_type_encoder_f LaneDirection_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _LaneDirection_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>
