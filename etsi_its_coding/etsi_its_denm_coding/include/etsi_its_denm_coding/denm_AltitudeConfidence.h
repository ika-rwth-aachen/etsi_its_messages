/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_AltitudeConfidence_H_
#define	_denm_AltitudeConfidence_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_AltitudeConfidence {
	denm_AltitudeConfidence_alt_000_01	= 0,
	denm_AltitudeConfidence_alt_000_02	= 1,
	denm_AltitudeConfidence_alt_000_05	= 2,
	denm_AltitudeConfidence_alt_000_10	= 3,
	denm_AltitudeConfidence_alt_000_20	= 4,
	denm_AltitudeConfidence_alt_000_50	= 5,
	denm_AltitudeConfidence_alt_001_00	= 6,
	denm_AltitudeConfidence_alt_002_00	= 7,
	denm_AltitudeConfidence_alt_005_00	= 8,
	denm_AltitudeConfidence_alt_010_00	= 9,
	denm_AltitudeConfidence_alt_020_00	= 10,
	denm_AltitudeConfidence_alt_050_00	= 11,
	denm_AltitudeConfidence_alt_100_00	= 12,
	denm_AltitudeConfidence_alt_200_00	= 13,
	denm_AltitudeConfidence_outOfRange	= 14,
	denm_AltitudeConfidence_unavailable	= 15
} e_denm_AltitudeConfidence;

/* denm_AltitudeConfidence */
typedef long	 denm_AltitudeConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_AltitudeConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_AltitudeConfidence;
extern const asn_INTEGER_specifics_t asn_SPC_denm_AltitudeConfidence_specs_1;
asn_struct_free_f denm_AltitudeConfidence_free;
asn_struct_print_f denm_AltitudeConfidence_print;
asn_constr_check_f denm_AltitudeConfidence_constraint;
jer_type_encoder_f denm_AltitudeConfidence_encode_jer;
per_type_decoder_f denm_AltitudeConfidence_decode_uper;
per_type_encoder_f denm_AltitudeConfidence_encode_uper;
per_type_decoder_f denm_AltitudeConfidence_decode_aper;
per_type_encoder_f denm_AltitudeConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_AltitudeConfidence_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
