/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_RelevanceTrafficDirection_H_
#define	_denm_RelevanceTrafficDirection_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_RelevanceTrafficDirection {
	denm_RelevanceTrafficDirection_allTrafficDirections	= 0,
	denm_RelevanceTrafficDirection_upstreamTraffic	= 1,
	denm_RelevanceTrafficDirection_downstreamTraffic	= 2,
	denm_RelevanceTrafficDirection_oppositeTraffic	= 3
} e_denm_RelevanceTrafficDirection;

/* denm_RelevanceTrafficDirection */
typedef long	 denm_RelevanceTrafficDirection_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_RelevanceTrafficDirection_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_RelevanceTrafficDirection;
extern const asn_INTEGER_specifics_t asn_SPC_denm_RelevanceTrafficDirection_specs_1;
asn_struct_free_f denm_RelevanceTrafficDirection_free;
asn_struct_print_f denm_RelevanceTrafficDirection_print;
asn_constr_check_f denm_RelevanceTrafficDirection_constraint;
jer_type_encoder_f denm_RelevanceTrafficDirection_encode_jer;
per_type_decoder_f denm_RelevanceTrafficDirection_decode_uper;
per_type_encoder_f denm_RelevanceTrafficDirection_encode_uper;
per_type_decoder_f denm_RelevanceTrafficDirection_decode_aper;
per_type_encoder_f denm_RelevanceTrafficDirection_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_RelevanceTrafficDirection_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
