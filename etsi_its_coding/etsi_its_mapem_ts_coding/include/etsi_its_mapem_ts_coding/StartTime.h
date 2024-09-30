/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in "/input/ISO14816_AVIAEINumberingAndDataStructures.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/AviEriDateTime.h"
#ifndef	_StartTime_H_
#define	_StartTime_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* StartTime */
typedef AviEriDateTime_t	 StartTime_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_StartTime_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_StartTime;
asn_struct_free_f StartTime_free;
asn_struct_print_f StartTime_print;
asn_constr_check_f StartTime_constraint;
ber_type_decoder_f StartTime_decode_ber;
der_type_encoder_f StartTime_encode_der;
xer_type_decoder_f StartTime_decode_xer;
xer_type_encoder_f StartTime_encode_xer;
jer_type_encoder_f StartTime_encode_jer;
oer_type_decoder_f StartTime_decode_oer;
oer_type_encoder_f StartTime_encode_oer;
per_type_decoder_f StartTime_decode_uper;
per_type_encoder_f StartTime_encode_uper;
per_type_decoder_f StartTime_decode_aper;
per_type_encoder_f StartTime_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _StartTime_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
