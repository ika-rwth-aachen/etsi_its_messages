/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/cdd/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#ifndef	_DangerousGoodsExtended_H_
#define	_DangerousGoodsExtended_H_


#include <asn_application.h>

/* Including external dependencies */
#include "DangerousGoodsBasic.h"
#include <NativeInteger.h>
#include <BOOLEAN.h>
#include <IA5String.h>
#include "PhoneNumber.h"
#include <UTF8String.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DangerousGoodsExtended */
typedef struct DangerousGoodsExtended {
	DangerousGoodsBasic_t	 dangerousGoodsType;
	long	 unNumber;
	BOOLEAN_t	 elevatedTemperature;
	BOOLEAN_t	 tunnelsRestricted;
	BOOLEAN_t	 limitedQuantity;
	IA5String_t	*emergencyActionCode	/* OPTIONAL */;
	PhoneNumber_t	*phoneNumber	/* OPTIONAL */;
	UTF8String_t	*companyName	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DangerousGoodsExtended_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DangerousGoodsExtended;
extern asn_SEQUENCE_specifics_t asn_SPC_DangerousGoodsExtended_specs_1;
extern asn_TYPE_member_t asn_MBR_DangerousGoodsExtended_1[8];

#ifdef __cplusplus
}
#endif

#endif	/* _DangerousGoodsExtended_H_ */
#include <asn_internal.h>
