/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_LaneAttributes_Vehicle_H_
#define	_spatem_ts_LaneAttributes_Vehicle_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum spatem_ts_LaneAttributes_Vehicle {
	spatem_ts_LaneAttributes_Vehicle_isVehicleRevocableLane	= 0,
	spatem_ts_LaneAttributes_Vehicle_isVehicleFlyOverLane	= 1,
	spatem_ts_LaneAttributes_Vehicle_hovLaneUseOnly	= 2,
	spatem_ts_LaneAttributes_Vehicle_restrictedToBusUse	= 3,
	spatem_ts_LaneAttributes_Vehicle_restrictedToTaxiUse	= 4,
	spatem_ts_LaneAttributes_Vehicle_restrictedFromPublicUse	= 5,
	spatem_ts_LaneAttributes_Vehicle_hasIRbeaconCoverage	= 6,
	spatem_ts_LaneAttributes_Vehicle_permissionOnRequest	= 7
} e_spatem_ts_LaneAttributes_Vehicle;

/* spatem_ts_LaneAttributes-Vehicle */
typedef BIT_STRING_t	 spatem_ts_LaneAttributes_Vehicle_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_spatem_ts_LaneAttributes_Vehicle_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_LaneAttributes_Vehicle;
asn_struct_free_f spatem_ts_LaneAttributes_Vehicle_free;
asn_struct_print_f spatem_ts_LaneAttributes_Vehicle_print;
asn_constr_check_f spatem_ts_LaneAttributes_Vehicle_constraint;
jer_type_encoder_f spatem_ts_LaneAttributes_Vehicle_encode_jer;
per_type_decoder_f spatem_ts_LaneAttributes_Vehicle_decode_uper;
per_type_encoder_f spatem_ts_LaneAttributes_Vehicle_encode_uper;
per_type_decoder_f spatem_ts_LaneAttributes_Vehicle_decode_aper;
per_type_encoder_f spatem_ts_LaneAttributes_Vehicle_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_LaneAttributes_Vehicle_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
