/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_cam_RelevanceDistance_H_
#define	_cam_RelevanceDistance_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_RelevanceDistance {
	cam_RelevanceDistance_lessThan50m	= 0,
	cam_RelevanceDistance_lessThan100m	= 1,
	cam_RelevanceDistance_lessThan200m	= 2,
	cam_RelevanceDistance_lessThan500m	= 3,
	cam_RelevanceDistance_lessThan1000m	= 4,
	cam_RelevanceDistance_lessThan5km	= 5,
	cam_RelevanceDistance_lessThan10km	= 6,
	cam_RelevanceDistance_over10km	= 7
} e_cam_RelevanceDistance;

/* cam_RelevanceDistance */
typedef long	 cam_RelevanceDistance_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_RelevanceDistance;
asn_struct_free_f cam_RelevanceDistance_free;
asn_struct_print_f cam_RelevanceDistance_print;
asn_constr_check_f cam_RelevanceDistance_constraint;
jer_type_encoder_f cam_RelevanceDistance_encode_jer;
per_type_decoder_f cam_RelevanceDistance_decode_uper;
per_type_encoder_f cam_RelevanceDistance_encode_uper;
per_type_decoder_f cam_RelevanceDistance_decode_aper;
per_type_encoder_f cam_RelevanceDistance_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_RelevanceDistance_H_ */
#include <etsi_its_cam_coding/asn_internal.h>
