/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_uulm_PositionOfOccupants_H_
#define	_mcm_uulm_PositionOfOccupants_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_PositionOfOccupants {
	mcm_uulm_PositionOfOccupants_row1LeftOccupied	= 0,
	mcm_uulm_PositionOfOccupants_row1RightOccupied	= 1,
	mcm_uulm_PositionOfOccupants_row1MidOccupied	= 2,
	mcm_uulm_PositionOfOccupants_row1NotDetectable	= 3,
	mcm_uulm_PositionOfOccupants_row1NotPresent	= 4,
	mcm_uulm_PositionOfOccupants_row2LeftOccupied	= 5,
	mcm_uulm_PositionOfOccupants_row2RightOccupied	= 6,
	mcm_uulm_PositionOfOccupants_row2MidOccupied	= 7,
	mcm_uulm_PositionOfOccupants_row2NotDetectable	= 8,
	mcm_uulm_PositionOfOccupants_row2NotPresent	= 9,
	mcm_uulm_PositionOfOccupants_row3LeftOccupied	= 10,
	mcm_uulm_PositionOfOccupants_row3RightOccupied	= 11,
	mcm_uulm_PositionOfOccupants_row3MidOccupied	= 12,
	mcm_uulm_PositionOfOccupants_row3NotDetectable	= 13,
	mcm_uulm_PositionOfOccupants_row3NotPresent	= 14,
	mcm_uulm_PositionOfOccupants_row4LeftOccupied	= 15,
	mcm_uulm_PositionOfOccupants_row4RightOccupied	= 16,
	mcm_uulm_PositionOfOccupants_row4MidOccupied	= 17,
	mcm_uulm_PositionOfOccupants_row4NotDetectable	= 18,
	mcm_uulm_PositionOfOccupants_row4NotPresent	= 19
} e_mcm_uulm_PositionOfOccupants;

/* mcm_uulm_PositionOfOccupants */
typedef BIT_STRING_t	 mcm_uulm_PositionOfOccupants_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_PositionOfOccupants;
asn_struct_free_f mcm_uulm_PositionOfOccupants_free;
asn_struct_print_f mcm_uulm_PositionOfOccupants_print;
asn_constr_check_f mcm_uulm_PositionOfOccupants_constraint;
per_type_decoder_f mcm_uulm_PositionOfOccupants_decode_uper;
per_type_encoder_f mcm_uulm_PositionOfOccupants_encode_uper;
per_type_decoder_f mcm_uulm_PositionOfOccupants_decode_aper;
per_type_encoder_f mcm_uulm_PositionOfOccupants_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_PositionOfOccupants_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
