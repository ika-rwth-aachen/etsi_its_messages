/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_mcm_ts_VruDeviceUsage_H_
#define	_mcm_ts_VruDeviceUsage_H_


#include <uulm_mcm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <uulm_mcm_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum mcm_ts_VruDeviceUsage {
	mcm_ts_VruDeviceUsage_unavailable	= 0,
	mcm_ts_VruDeviceUsage_other	= 1,
	mcm_ts_VruDeviceUsage_idle	= 2,
	mcm_ts_VruDeviceUsage_listeningToAudio	= 3,
	mcm_ts_VruDeviceUsage_typing	= 4,
	mcm_ts_VruDeviceUsage_calling	= 5,
	mcm_ts_VruDeviceUsage_playingGames	= 6,
	mcm_ts_VruDeviceUsage_reading	= 7,
	mcm_ts_VruDeviceUsage_viewing	= 8
} e_mcm_ts_VruDeviceUsage;

/* mcm_ts_VruDeviceUsage */
typedef long	 mcm_ts_VruDeviceUsage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_mcm_ts_VruDeviceUsage;
asn_struct_free_f mcm_ts_VruDeviceUsage_free;
asn_struct_print_f mcm_ts_VruDeviceUsage_print;
asn_constr_check_f mcm_ts_VruDeviceUsage_constraint;
per_type_decoder_f mcm_ts_VruDeviceUsage_decode_uper;
per_type_encoder_f mcm_ts_VruDeviceUsage_encode_uper;
per_type_decoder_f mcm_ts_VruDeviceUsage_decode_aper;
per_type_encoder_f mcm_ts_VruDeviceUsage_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _mcm_ts_VruDeviceUsage_H_ */
#include <uulm_mcm_ts_coding/asn_internal.h>
