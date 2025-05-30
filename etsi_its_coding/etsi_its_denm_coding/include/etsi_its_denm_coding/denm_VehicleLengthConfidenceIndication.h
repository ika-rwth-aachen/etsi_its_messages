/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER`
 */

#ifndef	_denm_VehicleLengthConfidenceIndication_H_
#define	_denm_VehicleLengthConfidenceIndication_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum denm_VehicleLengthConfidenceIndication {
	denm_VehicleLengthConfidenceIndication_noTrailerPresent	= 0,
	denm_VehicleLengthConfidenceIndication_trailerPresentWithKnownLength	= 1,
	denm_VehicleLengthConfidenceIndication_trailerPresentWithUnknownLength	= 2,
	denm_VehicleLengthConfidenceIndication_trailerPresenceIsUnknown	= 3,
	denm_VehicleLengthConfidenceIndication_unavailable	= 4
} e_denm_VehicleLengthConfidenceIndication;

/* denm_VehicleLengthConfidenceIndication */
typedef long	 denm_VehicleLengthConfidenceIndication_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_denm_VehicleLengthConfidenceIndication_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_denm_VehicleLengthConfidenceIndication;
extern const asn_INTEGER_specifics_t asn_SPC_denm_VehicleLengthConfidenceIndication_specs_1;
asn_struct_free_f denm_VehicleLengthConfidenceIndication_free;
asn_struct_print_f denm_VehicleLengthConfidenceIndication_print;
asn_constr_check_f denm_VehicleLengthConfidenceIndication_constraint;
jer_type_encoder_f denm_VehicleLengthConfidenceIndication_encode_jer;
per_type_decoder_f denm_VehicleLengthConfidenceIndication_decode_uper;
per_type_encoder_f denm_VehicleLengthConfidenceIndication_encode_uper;
per_type_decoder_f denm_VehicleLengthConfidenceIndication_decode_aper;
per_type_encoder_f denm_VehicleLengthConfidenceIndication_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _denm_VehicleLengthConfidenceIndication_H_ */
#include <etsi_its_denm_coding/asn_internal.h>
