/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_HeightLonCarr_H_
#define	_ivim_ts_HeightLonCarr_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_HeightLonCarr {
	ivim_ts_HeightLonCarr_oneCentimeter	= 1,
	ivim_ts_HeightLonCarr_unavailable	= 100
} e_ivim_ts_HeightLonCarr;

/* ivim_ts_HeightLonCarr */
typedef long	 ivim_ts_HeightLonCarr_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_HeightLonCarr;
asn_struct_free_f ivim_ts_HeightLonCarr_free;
asn_struct_print_f ivim_ts_HeightLonCarr_print;
asn_constr_check_f ivim_ts_HeightLonCarr_constraint;
per_type_decoder_f ivim_ts_HeightLonCarr_decode_uper;
per_type_encoder_f ivim_ts_HeightLonCarr_encode_uper;
per_type_decoder_f ivim_ts_HeightLonCarr_decode_aper;
per_type_encoder_f ivim_ts_HeightLonCarr_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_HeightLonCarr_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
