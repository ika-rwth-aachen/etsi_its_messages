/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_VruMovementControl_H_
#define	_mcm_ts_VruMovementControl_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_VruMovementControl {
	mcm_ts_VruMovementControl_unavailable	= 0,
	mcm_ts_VruMovementControl_braking	= 1,
	mcm_ts_VruMovementControl_hardBraking	= 2,
	mcm_ts_VruMovementControl_stopPedaling	= 3,
	mcm_ts_VruMovementControl_brakingAndStopPedaling	= 4,
	mcm_ts_VruMovementControl_hardBrakingAndStopPedaling	= 5,
	mcm_ts_VruMovementControl_noReaction	= 6
} e_mcm_ts_VruMovementControl;

/* mcm_ts_VruMovementControl */
typedef long	 mcm_ts_VruMovementControl_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_VruMovementControl;
asn_struct_free_f mcm_ts_VruMovementControl_free;
asn_struct_print_f mcm_ts_VruMovementControl_print;
asn_constr_check_f mcm_ts_VruMovementControl_constraint;
per_type_decoder_f mcm_ts_VruMovementControl_decode_uper;
per_type_encoder_f mcm_ts_VruMovementControl_encode_uper;
per_type_decoder_f mcm_ts_VruMovementControl_decode_aper;
per_type_encoder_f mcm_ts_VruMovementControl_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_VruMovementControl_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
