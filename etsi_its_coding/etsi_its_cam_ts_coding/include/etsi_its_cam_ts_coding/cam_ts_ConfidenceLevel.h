/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_ConfidenceLevel_H_
#define	_cam_ts_ConfidenceLevel_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_ConfidenceLevel {
	cam_ts_ConfidenceLevel_unavailable	= 101
} e_cam_ts_ConfidenceLevel;

/* cam_ts_ConfidenceLevel */
typedef long	 cam_ts_ConfidenceLevel_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_ts_ConfidenceLevel_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_ConfidenceLevel;
asn_struct_free_f cam_ts_ConfidenceLevel_free;
asn_struct_print_f cam_ts_ConfidenceLevel_print;
asn_constr_check_f cam_ts_ConfidenceLevel_constraint;
jer_type_encoder_f cam_ts_ConfidenceLevel_encode_jer;
per_type_decoder_f cam_ts_ConfidenceLevel_decode_uper;
per_type_encoder_f cam_ts_ConfidenceLevel_encode_uper;
per_type_decoder_f cam_ts_ConfidenceLevel_decode_aper;
per_type_encoder_f cam_ts_ConfidenceLevel_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_ConfidenceLevel_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
