/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "MCM-PDU-Descriptions"
 * 	found in "/input/TS103561_LUKAS_MCM.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_WaypointDeltaTime_H_
#define	_mcm_ts_WaypointDeltaTime_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_WaypointDeltaTime {
	mcm_ts_WaypointDeltaTime_oneMilliSec	= 1
} e_mcm_ts_WaypointDeltaTime;

/* mcm_ts_WaypointDeltaTime */
typedef long	 mcm_ts_WaypointDeltaTime_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_mcm_ts_WaypointDeltaTime_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_WaypointDeltaTime;
asn_struct_free_f mcm_ts_WaypointDeltaTime_free;
asn_struct_print_f mcm_ts_WaypointDeltaTime_print;
asn_constr_check_f mcm_ts_WaypointDeltaTime_constraint;
per_type_decoder_f mcm_ts_WaypointDeltaTime_decode_uper;
per_type_encoder_f mcm_ts_WaypointDeltaTime_encode_uper;
per_type_decoder_f mcm_ts_WaypointDeltaTime_decode_aper;
per_type_encoder_f mcm_ts_WaypointDeltaTime_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_WaypointDeltaTime_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
