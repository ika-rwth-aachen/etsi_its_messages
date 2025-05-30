/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_PositionOfOccupants_H_
#define	_mapem_ts_PositionOfOccupants_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_PositionOfOccupants {
	mapem_ts_PositionOfOccupants_row1LeftOccupied	= 0,
	mapem_ts_PositionOfOccupants_row1RightOccupied	= 1,
	mapem_ts_PositionOfOccupants_row1MidOccupied	= 2,
	mapem_ts_PositionOfOccupants_row1NotDetectable	= 3,
	mapem_ts_PositionOfOccupants_row1NotPresent	= 4,
	mapem_ts_PositionOfOccupants_row2LeftOccupied	= 5,
	mapem_ts_PositionOfOccupants_row2RightOccupied	= 6,
	mapem_ts_PositionOfOccupants_row2MidOccupied	= 7,
	mapem_ts_PositionOfOccupants_row2NotDetectable	= 8,
	mapem_ts_PositionOfOccupants_row2NotPresent	= 9,
	mapem_ts_PositionOfOccupants_row3LeftOccupied	= 10,
	mapem_ts_PositionOfOccupants_row3RightOccupied	= 11,
	mapem_ts_PositionOfOccupants_row3MidOccupied	= 12,
	mapem_ts_PositionOfOccupants_row3NotDetectable	= 13,
	mapem_ts_PositionOfOccupants_row3NotPresent	= 14,
	mapem_ts_PositionOfOccupants_row4LeftOccupied	= 15,
	mapem_ts_PositionOfOccupants_row4RightOccupied	= 16,
	mapem_ts_PositionOfOccupants_row4MidOccupied	= 17,
	mapem_ts_PositionOfOccupants_row4NotDetectable	= 18,
	mapem_ts_PositionOfOccupants_row4NotPresent	= 19
} e_mapem_ts_PositionOfOccupants;

/* mapem_ts_PositionOfOccupants */
typedef BIT_STRING_t	 mapem_ts_PositionOfOccupants_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_PositionOfOccupants;
asn_struct_free_f mapem_ts_PositionOfOccupants_free;
asn_struct_print_f mapem_ts_PositionOfOccupants_print;
asn_constr_check_f mapem_ts_PositionOfOccupants_constraint;
jer_type_encoder_f mapem_ts_PositionOfOccupants_encode_jer;
per_type_decoder_f mapem_ts_PositionOfOccupants_decode_uper;
per_type_encoder_f mapem_ts_PositionOfOccupants_encode_uper;
per_type_decoder_f mapem_ts_PositionOfOccupants_decode_aper;
per_type_encoder_f mapem_ts_PositionOfOccupants_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_PositionOfOccupants_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
