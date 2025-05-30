/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_StationType_H_
#define	_spatem_ts_StationType_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_StationType {
	spatem_ts_StationType_unknown	= 0,
	spatem_ts_StationType_pedestrian	= 1,
	spatem_ts_StationType_cyclist	= 2,
	spatem_ts_StationType_moped	= 3,
	spatem_ts_StationType_motorcycle	= 4,
	spatem_ts_StationType_passengerCar	= 5,
	spatem_ts_StationType_bus	= 6,
	spatem_ts_StationType_lightTruck	= 7,
	spatem_ts_StationType_heavyTruck	= 8,
	spatem_ts_StationType_trailer	= 9,
	spatem_ts_StationType_specialVehicles	= 10,
	spatem_ts_StationType_tram	= 11,
	spatem_ts_StationType_roadSideUnit	= 15
} e_spatem_ts_StationType;

/* spatem_ts_StationType */
typedef long	 spatem_ts_StationType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_StationType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_StationType;
asn_struct_free_f spatem_ts_StationType_free;
asn_struct_print_f spatem_ts_StationType_print;
asn_constr_check_f spatem_ts_StationType_constraint;
jer_type_encoder_f spatem_ts_StationType_encode_jer;
per_type_decoder_f spatem_ts_StationType_decode_uper;
per_type_encoder_f spatem_ts_StationType_encode_uper;
per_type_decoder_f spatem_ts_StationType_decode_aper;
per_type_encoder_f spatem_ts_StationType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_StationType_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
