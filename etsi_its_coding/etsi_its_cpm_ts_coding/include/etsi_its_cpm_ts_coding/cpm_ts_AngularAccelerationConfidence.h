/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cpm_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cpm_ts_AngularAccelerationConfidence_H_
#define	_cpm_ts_AngularAccelerationConfidence_H_


#include <etsi_its_cpm_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cpm_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cpm_ts_AngularAccelerationConfidence {
	cpm_ts_AngularAccelerationConfidence_degSecSquared_01	= 0,
	cpm_ts_AngularAccelerationConfidence_degSecSquared_02	= 1,
	cpm_ts_AngularAccelerationConfidence_degSecSquared_05	= 2,
	cpm_ts_AngularAccelerationConfidence_degSecSquared_10	= 3,
	cpm_ts_AngularAccelerationConfidence_degSecSquared_20	= 4,
	cpm_ts_AngularAccelerationConfidence_degSecSquared_50	= 5,
	cpm_ts_AngularAccelerationConfidence_outOfRange	= 6,
	cpm_ts_AngularAccelerationConfidence_unavailable	= 7
} e_cpm_ts_AngularAccelerationConfidence;

/* cpm_ts_AngularAccelerationConfidence */
typedef long	 cpm_ts_AngularAccelerationConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cpm_ts_AngularAccelerationConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cpm_ts_AngularAccelerationConfidence;
extern const asn_INTEGER_specifics_t asn_SPC_cpm_ts_AngularAccelerationConfidence_specs_1;
asn_struct_free_f cpm_ts_AngularAccelerationConfidence_free;
asn_struct_print_f cpm_ts_AngularAccelerationConfidence_print;
asn_constr_check_f cpm_ts_AngularAccelerationConfidence_constraint;
jer_type_encoder_f cpm_ts_AngularAccelerationConfidence_encode_jer;
per_type_decoder_f cpm_ts_AngularAccelerationConfidence_decode_uper;
per_type_encoder_f cpm_ts_AngularAccelerationConfidence_encode_uper;
per_type_decoder_f cpm_ts_AngularAccelerationConfidence_decode_aper;
per_type_encoder_f cpm_ts_AngularAccelerationConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cpm_ts_AngularAccelerationConfidence_H_ */
#include <etsi_its_cpm_ts_coding/asn_internal.h>
