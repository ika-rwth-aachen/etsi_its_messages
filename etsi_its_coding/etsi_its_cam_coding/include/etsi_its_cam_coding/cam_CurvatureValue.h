/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_CurvatureValue_H_
#define	_cam_CurvatureValue_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_CurvatureValue {
	cam_CurvatureValue_straight	= 0,
	cam_CurvatureValue_unavailable	= 1023
} e_cam_CurvatureValue;

/* cam_CurvatureValue */
typedef long	 cam_CurvatureValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_CurvatureValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_CurvatureValue;
asn_struct_free_f cam_CurvatureValue_free;
asn_struct_print_f cam_CurvatureValue_print;
asn_constr_check_f cam_CurvatureValue_constraint;
jer_type_encoder_f cam_CurvatureValue_encode_jer;
per_type_decoder_f cam_CurvatureValue_decode_uper;
per_type_encoder_f cam_CurvatureValue_encode_uper;
per_type_decoder_f cam_CurvatureValue_decode_aper;
per_type_encoder_f cam_CurvatureValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_CurvatureValue_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
