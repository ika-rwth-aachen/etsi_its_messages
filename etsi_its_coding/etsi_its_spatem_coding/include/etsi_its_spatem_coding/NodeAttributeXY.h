/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_spatem_coding/NativeEnumerated.h>
#ifndef	_NodeAttributeXY_H_
#define	_NodeAttributeXY_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NodeAttributeXY {
	NodeAttributeXY_reserved	= 0,
	NodeAttributeXY_stopLine	= 1,
	NodeAttributeXY_roundedCapStyleA	= 2,
	NodeAttributeXY_roundedCapStyleB	= 3,
	NodeAttributeXY_mergePoint	= 4,
	NodeAttributeXY_divergePoint	= 5,
	NodeAttributeXY_downstreamStopLine	= 6,
	NodeAttributeXY_downstreamStartNode	= 7,
	NodeAttributeXY_closedToTraffic	= 8,
	NodeAttributeXY_safeIsland	= 9,
	NodeAttributeXY_curbPresentAtStepOff	= 10,
	NodeAttributeXY_hydrantPresent	= 11
	/*
	 * Enumeration is extensible
	 */
} e_NodeAttributeXY;

/* NodeAttributeXY */
typedef long	 NodeAttributeXY_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_NodeAttributeXY_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_NodeAttributeXY;
extern const asn_INTEGER_specifics_t asn_SPC_NodeAttributeXY_specs_1;
asn_struct_free_f NodeAttributeXY_free;
asn_struct_print_f NodeAttributeXY_print;
asn_constr_check_f NodeAttributeXY_constraint;
ber_type_decoder_f NodeAttributeXY_decode_ber;
der_type_encoder_f NodeAttributeXY_encode_der;
xer_type_decoder_f NodeAttributeXY_decode_xer;
xer_type_encoder_f NodeAttributeXY_encode_xer;
jer_type_encoder_f NodeAttributeXY_encode_jer;
oer_type_decoder_f NodeAttributeXY_decode_oer;
oer_type_encoder_f NodeAttributeXY_encode_oer;
per_type_decoder_f NodeAttributeXY_decode_uper;
per_type_encoder_f NodeAttributeXY_encode_uper;
per_type_decoder_f NodeAttributeXY_decode_aper;
per_type_encoder_f NodeAttributeXY_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _NodeAttributeXY_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
