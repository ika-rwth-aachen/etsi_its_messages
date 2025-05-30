/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=spatem_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_spatem_ts_WaitOnStopline_H_
#define	_spatem_ts_WaitOnStopline_H_


#include <etsi_its_spatem_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_spatem_ts_coding/BOOLEAN.h>

#ifdef __cplusplus
extern "C" {
#endif

/* spatem_ts_WaitOnStopline */
typedef BOOLEAN_t	 spatem_ts_WaitOnStopline_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_spatem_ts_WaitOnStopline;
asn_struct_free_f spatem_ts_WaitOnStopline_free;
asn_struct_print_f spatem_ts_WaitOnStopline_print;
asn_constr_check_f spatem_ts_WaitOnStopline_constraint;
jer_type_encoder_f spatem_ts_WaitOnStopline_encode_jer;
per_type_decoder_f spatem_ts_WaitOnStopline_decode_uper;
per_type_encoder_f spatem_ts_WaitOnStopline_encode_uper;
per_type_decoder_f spatem_ts_WaitOnStopline_decode_aper;
per_type_encoder_f spatem_ts_WaitOnStopline_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _spatem_ts_WaitOnStopline_H_ */
#include <etsi_its_spatem_ts_coding/asn_internal.h>
