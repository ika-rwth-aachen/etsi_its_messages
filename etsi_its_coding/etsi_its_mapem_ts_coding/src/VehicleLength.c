/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_mapem_ts_coding/VehicleLength.h"

static asn_TYPE_member_t asn_MBR_VehicleLength_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleLength, vehicleLengthValue),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleLengthValue,
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
		"vehicleLengthValue"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleLength, vehicleLengthConfidenceIndication),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleLengthConfidenceIndication,
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
		"vehicleLengthConfidenceIndication"
		},
};
static const ber_tlv_tag_t asn_DEF_VehicleLength_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleLength_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* vehicleLengthValue */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* vehicleLengthConfidenceIndication */
};
static asn_SEQUENCE_specifics_t asn_SPC_VehicleLength_specs_1 = {
	sizeof(struct VehicleLength),
	offsetof(struct VehicleLength, _asn_ctx),
	asn_MAP_VehicleLength_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VehicleLength = {
	"VehicleLength",
	"VehicleLength",
	&asn_OP_SEQUENCE,
	asn_DEF_VehicleLength_tags_1,
	sizeof(asn_DEF_VehicleLength_tags_1)
		/sizeof(asn_DEF_VehicleLength_tags_1[0]), /* 1 */
	asn_DEF_VehicleLength_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleLength_tags_1)
		/sizeof(asn_DEF_VehicleLength_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_VehicleLength_1,
	2,	/* Elements count */
	&asn_SPC_VehicleLength_specs_1	/* Additional specs */
};

