/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cam_ts_CurvatureValue_H_
#define	_cam_ts_CurvatureValue_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_CurvatureValue {
	cam_ts_CurvatureValue_outOfRangeNegative	= -1023,
	cam_ts_CurvatureValue_straight	= 0,
	cam_ts_CurvatureValue_outOfRangePositive	= 1022,
	cam_ts_CurvatureValue_unavailable	= 1023
} e_cam_ts_CurvatureValue;

/* cam_ts_CurvatureValue */
typedef long	 cam_ts_CurvatureValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_ts_CurvatureValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_CurvatureValue;
asn_struct_free_f cam_ts_CurvatureValue_free;
asn_struct_print_f cam_ts_CurvatureValue_print;
asn_constr_check_f cam_ts_CurvatureValue_constraint;
per_type_decoder_f cam_ts_CurvatureValue_decode_uper;
per_type_encoder_f cam_ts_CurvatureValue_encode_uper;
per_type_decoder_f cam_ts_CurvatureValue_decode_aper;
per_type_encoder_f cam_ts_CurvatureValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_CurvatureValue_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>