/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_spatem_ts_PositionOfOccupants_H_
#define	_spatem_ts_PositionOfOccupants_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_PositionOfOccupants {
	spatem_ts_PositionOfOccupants_row1LeftOccupied	= 0,
	spatem_ts_PositionOfOccupants_row1RightOccupied	= 1,
	spatem_ts_PositionOfOccupants_row1MidOccupied	= 2,
	spatem_ts_PositionOfOccupants_row1NotDetectable	= 3,
	spatem_ts_PositionOfOccupants_row1NotPresent	= 4,
	spatem_ts_PositionOfOccupants_row2LeftOccupied	= 5,
	spatem_ts_PositionOfOccupants_row2RightOccupied	= 6,
	spatem_ts_PositionOfOccupants_row2MidOccupied	= 7,
	spatem_ts_PositionOfOccupants_row2NotDetectable	= 8,
	spatem_ts_PositionOfOccupants_row2NotPresent	= 9,
	spatem_ts_PositionOfOccupants_row3LeftOccupied	= 10,
	spatem_ts_PositionOfOccupants_row3RightOccupied	= 11,
	spatem_ts_PositionOfOccupants_row3MidOccupied	= 12,
	spatem_ts_PositionOfOccupants_row3NotDetectable	= 13,
	spatem_ts_PositionOfOccupants_row3NotPresent	= 14,
	spatem_ts_PositionOfOccupants_row4LeftOccupied	= 15,
	spatem_ts_PositionOfOccupants_row4RightOccupied	= 16,
	spatem_ts_PositionOfOccupants_row4MidOccupied	= 17,
	spatem_ts_PositionOfOccupants_row4NotDetectable	= 18,
	spatem_ts_PositionOfOccupants_row4NotPresent	= 19
} e_spatem_ts_PositionOfOccupants;

/* spatem_ts_PositionOfOccupants */
typedef BIT_STRING_t	 spatem_ts_PositionOfOccupants_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_PositionOfOccupants;
asn_struct_free_f spatem_ts_PositionOfOccupants_free;
asn_struct_print_f spatem_ts_PositionOfOccupants_print;
asn_constr_check_f spatem_ts_PositionOfOccupants_constraint;
per_type_decoder_f spatem_ts_PositionOfOccupants_decode_uper;
per_type_encoder_f spatem_ts_PositionOfOccupants_encode_uper;
per_type_decoder_f spatem_ts_PositionOfOccupants_decode_aper;
per_type_encoder_f spatem_ts_PositionOfOccupants_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_PositionOfOccupants_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>