/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ts_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_ts_AltitudeConfidence_H_
#define	_cam_ts_AltitudeConfidence_H_


#include <etsi_its_cam_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_ts_AltitudeConfidence {
	cam_ts_AltitudeConfidence_alt_000_01	= 0,
	cam_ts_AltitudeConfidence_alt_000_02	= 1,
	cam_ts_AltitudeConfidence_alt_000_05	= 2,
	cam_ts_AltitudeConfidence_alt_000_10	= 3,
	cam_ts_AltitudeConfidence_alt_000_20	= 4,
	cam_ts_AltitudeConfidence_alt_000_50	= 5,
	cam_ts_AltitudeConfidence_alt_001_00	= 6,
	cam_ts_AltitudeConfidence_alt_002_00	= 7,
	cam_ts_AltitudeConfidence_alt_005_00	= 8,
	cam_ts_AltitudeConfidence_alt_010_00	= 9,
	cam_ts_AltitudeConfidence_alt_020_00	= 10,
	cam_ts_AltitudeConfidence_alt_050_00	= 11,
	cam_ts_AltitudeConfidence_alt_100_00	= 12,
	cam_ts_AltitudeConfidence_alt_200_00	= 13,
	cam_ts_AltitudeConfidence_outOfRange	= 14,
	cam_ts_AltitudeConfidence_unavailable	= 15
} e_cam_ts_AltitudeConfidence;

/* cam_ts_AltitudeConfidence */
typedef long	 cam_ts_AltitudeConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_cam_ts_AltitudeConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_cam_ts_AltitudeConfidence;
extern const asn_INTEGER_specifics_t asn_SPC_cam_ts_AltitudeConfidence_specs_1;
asn_struct_free_f cam_ts_AltitudeConfidence_free;
asn_struct_print_f cam_ts_AltitudeConfidence_print;
asn_constr_check_f cam_ts_AltitudeConfidence_constraint;
jer_type_encoder_f cam_ts_AltitudeConfidence_encode_jer;
per_type_decoder_f cam_ts_AltitudeConfidence_decode_uper;
per_type_encoder_f cam_ts_AltitudeConfidence_encode_uper;
per_type_decoder_f cam_ts_AltitudeConfidence_decode_aper;
per_type_encoder_f cam_ts_AltitudeConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_ts_AltitudeConfidence_H_ */
#include <etsi_its_cam_ts_coding/asn_internal.h>
