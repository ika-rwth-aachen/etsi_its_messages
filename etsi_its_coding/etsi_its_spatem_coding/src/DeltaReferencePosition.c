/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_spatem_coding/DeltaReferencePosition.h"

asn_TYPE_member_t asn_MBR_DeltaReferencePosition_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DeltaReferencePosition, deltaLatitude),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DeltaLatitude,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"deltaLatitude"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DeltaReferencePosition, deltaLongitude),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DeltaLongitude,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"deltaLongitude"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DeltaReferencePosition, deltaAltitude),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DeltaAltitude,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"deltaAltitude"
		},
};
static const ber_tlv_tag_t asn_DEF_DeltaReferencePosition_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DeltaReferencePosition_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* deltaLatitude */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* deltaLongitude */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* deltaAltitude */
};
asn_SEQUENCE_specifics_t asn_SPC_DeltaReferencePosition_specs_1 = {
	sizeof(struct DeltaReferencePosition),
	offsetof(struct DeltaReferencePosition, _asn_ctx),
	asn_MAP_DeltaReferencePosition_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DeltaReferencePosition = {
	"DeltaReferencePosition",
	"DeltaReferencePosition",
	&asn_OP_SEQUENCE,
	asn_DEF_DeltaReferencePosition_tags_1,
	sizeof(asn_DEF_DeltaReferencePosition_tags_1)
		/sizeof(asn_DEF_DeltaReferencePosition_tags_1[0]), /* 1 */
	asn_DEF_DeltaReferencePosition_tags_1,	/* Same as above */
	sizeof(asn_DEF_DeltaReferencePosition_tags_1)
		/sizeof(asn_DEF_DeltaReferencePosition_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_DeltaReferencePosition_1,
	3,	/* Elements count */
	&asn_SPC_DeltaReferencePosition_specs_1	/* Additional specs */
};

