/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_LateralAccelerationValue_H_
#define	_cam_LateralAccelerationValue_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_LateralAccelerationValue {
	cam_LateralAccelerationValue_pointOneMeterPerSecSquaredToRight	= -1,
	cam_LateralAccelerationValue_pointOneMeterPerSecSquaredToLeft	= 1,
	cam_LateralAccelerationValue_unavailable	= 161
} e_cam_LateralAccelerationValue;

/* cam_LateralAccelerationValue */
typedef long	 cam_LateralAccelerationValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_LateralAccelerationValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_LateralAccelerationValue;
asn_struct_free_f cam_LateralAccelerationValue_free;
asn_struct_print_f cam_LateralAccelerationValue_print;
asn_constr_check_f cam_LateralAccelerationValue_constraint;
jer_type_encoder_f cam_LateralAccelerationValue_encode_jer;
per_type_decoder_f cam_LateralAccelerationValue_decode_uper;
per_type_encoder_f cam_LateralAccelerationValue_encode_uper;
per_type_decoder_f cam_LateralAccelerationValue_decode_aper;
per_type_encoder_f cam_LateralAccelerationValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_LateralAccelerationValue_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
