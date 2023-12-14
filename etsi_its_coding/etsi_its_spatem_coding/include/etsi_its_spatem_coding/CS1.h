/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in "/input/ISO14816_AVIAEINumberingAndDataStructures.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/CountryCode.h"
#include "etsi_its_spatem_coding/IssuerIdentifier.h"
#include "etsi_its_spatem_coding/ServiceNumber.h"
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_CS1_H_
#define	_CS1_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CS1 */
typedef struct CS1 {
	CountryCode_t	 countryCode;
	IssuerIdentifier_t	 issuerIdentifier;
	ServiceNumber_t	 serviceNumber;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CS1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CS1;

#ifdef __cplusplus
}
#endif

#endif	/* _CS1_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>
