/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ASN1C-UsefulInformationObjectClasses"
 * 	found in "/usr/local/share/asn1c/standard-modules/ASN1C-UsefulInformationObjectClasses.asn1"
 * 	`asn1c -fcompound-names -fprefix=cam_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_cam_EXTERNAL_H_
#define	_cam_EXTERNAL_H_


#include <etsi_its_cam_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_cam_coding/OBJECT_IDENTIFIER.h>
#include <etsi_its_cam_coding/NativeInteger.h>
#include <etsi_its_cam_coding/ObjectDescriptor.h>
#include <etsi_its_cam_coding/ANY.h>
#include <etsi_its_cam_coding/OCTET_STRING.h>
#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_cam_coding/constr_CHOICE.h>
#include <etsi_its_cam_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum cam_EXTERNAL__encoding_PR {
	cam_EXTERNAL__encoding_PR_NOTHING,	/* No components present */
	cam_EXTERNAL__encoding_PR_single_ASN1_type,
	cam_EXTERNAL__encoding_PR_octet_aligned,
	cam_EXTERNAL__encoding_PR_arbitrary
} cam_EXTERNAL__encoding_PR;

/* cam_EXTERNAL */
typedef struct cam_EXTERNAL {
	OBJECT_IDENTIFIER_t	*direct_reference;	/* OPTIONAL */
	long	*indirect_reference;	/* OPTIONAL */
	ObjectDescriptor_t	*data_value_descriptor;	/* OPTIONAL */
	struct cam_EXTERNAL__encoding {
		cam_EXTERNAL__encoding_PR present;
		union cam_EXTERNAL__cam_encoding_u {
			ANY_t	 single_ASN1_type;
			OCTET_STRING_t	 octet_aligned;
			BIT_STRING_t	 arbitrary;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} encoding;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} cam_EXTERNAL_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_cam_EXTERNAL;

#ifdef __cplusplus
}
#endif

#endif	/* _cam_EXTERNAL_H_ */
#include <etsi_its_cam_coding/asn_internal.h>