/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_TrafficConditionSubCauseCode_H_
#define	_cam_ts_TrafficConditionSubCauseCode_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_TrafficConditionSubCauseCode {
	cam_ts_TrafficConditionSubCauseCode_unavailable	= 0,
	cam_ts_TrafficConditionSubCauseCode_increasedVolumeOfTraffic	= 1,
	cam_ts_TrafficConditionSubCauseCode_trafficJamSlowlyIncreasing	= 2,
	cam_ts_TrafficConditionSubCauseCode_trafficJamIncreasing	= 3,
	cam_ts_TrafficConditionSubCauseCode_trafficJamStronglyIncreasing	= 4,
	cam_ts_TrafficConditionSubCauseCode_trafficJam	= 5,
	cam_ts_TrafficConditionSubCauseCode_trafficJamSlightlyDecreasing	= 6,
	cam_ts_TrafficConditionSubCauseCode_trafficJamDecreasing	= 7,
	cam_ts_TrafficConditionSubCauseCode_trafficJamStronglyDecreasing	= 8,
	cam_ts_TrafficConditionSubCauseCode_trafficJamStable	= 9
} e_cam_ts_TrafficConditionSubCauseCode;

/* cam_ts_TrafficConditionSubCauseCode */
typedef long	 cam_ts_TrafficConditionSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_ts_TrafficConditionSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_TrafficConditionSubCauseCode;
asn_struct_free_f cam_ts_TrafficConditionSubCauseCode_free;
asn_struct_print_f cam_ts_TrafficConditionSubCauseCode_print;
asn_constr_check_f cam_ts_TrafficConditionSubCauseCode_constraint;
jer_type_encoder_f cam_ts_TrafficConditionSubCauseCode_encode_jer;
per_type_decoder_f cam_ts_TrafficConditionSubCauseCode_decode_uper;
per_type_encoder_f cam_ts_TrafficConditionSubCauseCode_encode_uper;
per_type_decoder_f cam_ts_TrafficConditionSubCauseCode_decode_aper;
per_type_encoder_f cam_ts_TrafficConditionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_TrafficConditionSubCauseCode_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
