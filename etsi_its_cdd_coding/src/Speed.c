/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-PER`
 */

#include "Speed.h"

static asn_TYPE_member_t asn_MBR_Speed_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct Speed, speedValue),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SpeedValue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"speedValue"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Speed, speedConfidence),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SpeedConfidence,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"speedConfidence"
		},
};
static const ber_tlv_tag_t asn_DEF_Speed_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Speed_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* speedValue */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* speedConfidence */
};
static asn_SEQUENCE_specifics_t asn_SPC_Speed_specs_1 = {
	sizeof(struct Speed),
	offsetof(struct Speed, _asn_ctx),
	asn_MAP_Speed_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_Speed = {
	"Speed",
	"Speed",
	&asn_OP_SEQUENCE,
	asn_DEF_Speed_tags_1,
	sizeof(asn_DEF_Speed_tags_1)
		/sizeof(asn_DEF_Speed_tags_1[0]), /* 1 */
	asn_DEF_Speed_tags_1,	/* Same as above */
	sizeof(asn_DEF_Speed_tags_1)
		/sizeof(asn_DEF_Speed_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_Speed_1,
	2,	/* Elements count */
	&asn_SPC_Speed_specs_1	/* Additional specs */
};

