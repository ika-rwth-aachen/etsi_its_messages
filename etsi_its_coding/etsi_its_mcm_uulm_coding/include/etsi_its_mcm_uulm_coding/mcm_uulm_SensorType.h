/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_mcm_uulm_SensorType_H_
#define	_mcm_uulm_SensorType_H_


#include <etsi_its_mcm_uulm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_mcm_uulm_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_uulm_SensorType {
	mcm_uulm_SensorType_undefined	= 0,
	mcm_uulm_SensorType_radar	= 1,
	mcm_uulm_SensorType_lidar	= 2,
	mcm_uulm_SensorType_monovideo	= 3,
	mcm_uulm_SensorType_stereovision	= 4,
	mcm_uulm_SensorType_nightvision	= 5,
	mcm_uulm_SensorType_ultrasonic	= 6,
	mcm_uulm_SensorType_pmd	= 7,
	mcm_uulm_SensorType_inductionLoop	= 8,
	mcm_uulm_SensorType_sphericalCamera	= 9,
	mcm_uulm_SensorType_uwb	= 10,
	mcm_uulm_SensorType_acoustic	= 11,
	mcm_uulm_SensorType_localAggregation	= 12,
	mcm_uulm_SensorType_itsAggregation	= 13
} e_mcm_uulm_SensorType;

/* mcm_uulm_SensorType */
typedef long	 mcm_uulm_SensorType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_uulm_SensorType;
asn_struct_free_f mcm_uulm_SensorType_free;
asn_struct_print_f mcm_uulm_SensorType_print;
asn_constr_check_f mcm_uulm_SensorType_constraint;
jer_type_encoder_f mcm_uulm_SensorType_encode_jer;
per_type_decoder_f mcm_uulm_SensorType_decode_uper;
per_type_encoder_f mcm_uulm_SensorType_encode_uper;
per_type_decoder_f mcm_uulm_SensorType_decode_aper;
per_type_encoder_f mcm_uulm_SensorType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_uulm_SensorType_H_ */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
