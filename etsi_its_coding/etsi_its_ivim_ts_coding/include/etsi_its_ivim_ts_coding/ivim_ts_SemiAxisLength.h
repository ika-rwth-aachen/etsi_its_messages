/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_SemiAxisLength_H_
#define	_ivim_ts_SemiAxisLength_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_SemiAxisLength {
	ivim_ts_SemiAxisLength_oneCentimeter	= 1,
	ivim_ts_SemiAxisLength_outOfRange	= 4094,
	ivim_ts_SemiAxisLength_unavailable	= 4095
} e_ivim_ts_SemiAxisLength;

/* ivim_ts_SemiAxisLength */
typedef long	 ivim_ts_SemiAxisLength_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_SemiAxisLength_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_SemiAxisLength;
asn_struct_free_f ivim_ts_SemiAxisLength_free;
asn_struct_print_f ivim_ts_SemiAxisLength_print;
asn_constr_check_f ivim_ts_SemiAxisLength_constraint;
per_type_decoder_f ivim_ts_SemiAxisLength_decode_uper;
per_type_encoder_f ivim_ts_SemiAxisLength_encode_uper;
per_type_decoder_f ivim_ts_SemiAxisLength_decode_aper;
per_type_encoder_f ivim_ts_SemiAxisLength_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_SemiAxisLength_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
