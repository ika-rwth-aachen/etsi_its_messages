/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_LaneAttributes_TrackedVehicle_H_
#define	_mapem_ts_LaneAttributes_TrackedVehicle_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_LaneAttributes_TrackedVehicle {
	mapem_ts_LaneAttributes_TrackedVehicle_spec_RevocableLane	= 0,
	mapem_ts_LaneAttributes_TrackedVehicle_spec_commuterRailRoadTrack	= 1,
	mapem_ts_LaneAttributes_TrackedVehicle_spec_lightRailRoadTrack	= 2,
	mapem_ts_LaneAttributes_TrackedVehicle_spec_heavyRailRoadTrack	= 3,
	mapem_ts_LaneAttributes_TrackedVehicle_spec_otherRailType	= 4
} e_mapem_ts_LaneAttributes_TrackedVehicle;

/* mapem_ts_LaneAttributes-TrackedVehicle */
typedef BIT_STRING_t	 mapem_ts_LaneAttributes_TrackedVehicle_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mapem_ts_LaneAttributes_TrackedVehicle_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_LaneAttributes_TrackedVehicle;
asn_struct_free_f mapem_ts_LaneAttributes_TrackedVehicle_free;
asn_struct_print_f mapem_ts_LaneAttributes_TrackedVehicle_print;
asn_constr_check_f mapem_ts_LaneAttributes_TrackedVehicle_constraint;
jer_type_encoder_f mapem_ts_LaneAttributes_TrackedVehicle_encode_jer;
per_type_decoder_f mapem_ts_LaneAttributes_TrackedVehicle_decode_uper;
per_type_encoder_f mapem_ts_LaneAttributes_TrackedVehicle_encode_uper;
per_type_decoder_f mapem_ts_LaneAttributes_TrackedVehicle_decode_aper;
per_type_encoder_f mapem_ts_LaneAttributes_TrackedVehicle_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_LaneAttributes_TrackedVehicle_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
