/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_mapem_coding/BIT_STRING.h>
#ifndef	_AllowedManeuvers_H_
#define	_AllowedManeuvers_H_


#include <etsi_its_mapem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AllowedManeuvers {
	AllowedManeuvers_maneuverStraightAllowed	= 0,
	AllowedManeuvers_maneuverLeftAllowed	= 1,
	AllowedManeuvers_maneuverRightAllowed	= 2,
	AllowedManeuvers_maneuverUTurnAllowed	= 3,
	AllowedManeuvers_maneuverLeftTurnOnRedAllowed	= 4,
	AllowedManeuvers_maneuverRightTurnOnRedAllowed	= 5,
	AllowedManeuvers_maneuverLaneChangeAllowed	= 6,
	AllowedManeuvers_maneuverNoStoppingAllowed	= 7,
	AllowedManeuvers_yieldAllwaysRequired	= 8,
	AllowedManeuvers_goWithHalt	= 9,
	AllowedManeuvers_caution	= 10,
	AllowedManeuvers_reserved1	= 11
} e_AllowedManeuvers;

/* AllowedManeuvers */
typedef BIT_STRING_t	 AllowedManeuvers_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_AllowedManeuvers_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_AllowedManeuvers;
asn_struct_free_f AllowedManeuvers_free;
asn_struct_print_f AllowedManeuvers_print;
asn_constr_check_f AllowedManeuvers_constraint;
ber_type_decoder_f AllowedManeuvers_decode_ber;
der_type_encoder_f AllowedManeuvers_encode_der;
xer_type_decoder_f AllowedManeuvers_decode_xer;
xer_type_encoder_f AllowedManeuvers_encode_xer;
jer_type_encoder_f AllowedManeuvers_encode_jer;
oer_type_decoder_f AllowedManeuvers_decode_oer;
oer_type_encoder_f AllowedManeuvers_encode_oer;
per_type_decoder_f AllowedManeuvers_decode_uper;
per_type_encoder_f AllowedManeuvers_encode_uper;
per_type_decoder_f AllowedManeuvers_decode_aper;
per_type_encoder_f AllowedManeuvers_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _AllowedManeuvers_H_ */
#include <etsi_its_mapem_coding/asn_internal.h>