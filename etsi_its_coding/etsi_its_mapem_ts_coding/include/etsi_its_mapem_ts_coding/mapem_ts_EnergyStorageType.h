/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=mapem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mapem_ts_EnergyStorageType_H_
#define	_mapem_ts_EnergyStorageType_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mapem_ts_coding/BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mapem_ts_EnergyStorageType {
	mapem_ts_EnergyStorageType_hydrogenStorage	= 0,
	mapem_ts_EnergyStorageType_electricEnergyStorage	= 1,
	mapem_ts_EnergyStorageType_liquidPropaneGas	= 2,
	mapem_ts_EnergyStorageType_compressedNaturalGas	= 3,
	mapem_ts_EnergyStorageType_diesel	= 4,
	mapem_ts_EnergyStorageType_gasoline	= 5,
	mapem_ts_EnergyStorageType_ammonia	= 6
} e_mapem_ts_EnergyStorageType;

/* mapem_ts_EnergyStorageType */
typedef BIT_STRING_t	 mapem_ts_EnergyStorageType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mapem_ts_EnergyStorageType;
asn_struct_free_f mapem_ts_EnergyStorageType_free;
asn_struct_print_f mapem_ts_EnergyStorageType_print;
asn_constr_check_f mapem_ts_EnergyStorageType_constraint;
jer_type_encoder_f mapem_ts_EnergyStorageType_encode_jer;
per_type_decoder_f mapem_ts_EnergyStorageType_decode_uper;
per_type_encoder_f mapem_ts_EnergyStorageType_encode_uper;
per_type_decoder_f mapem_ts_EnergyStorageType_decode_aper;
per_type_encoder_f mapem_ts_EnergyStorageType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mapem_ts_EnergyStorageType_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
