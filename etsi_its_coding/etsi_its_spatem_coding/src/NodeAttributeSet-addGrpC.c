/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_spatem_coding/NodeAttributeSet-addGrpC.h"

asn_TYPE_member_t asn_MBR_NodeAttributeSet_addGrpC_1[] = {
	{ ATF_POINTER, 3, offsetof(struct NodeAttributeSet_addGrpC, ptvRequest),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PtvRequestType,
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
		"ptvRequest"
		},
	{ ATF_POINTER, 2, offsetof(struct NodeAttributeSet_addGrpC, nodeLink),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NodeLink,
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
		"nodeLink"
		},
	{ ATF_POINTER, 1, offsetof(struct NodeAttributeSet_addGrpC, node),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Node,
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
		"node"
		},
};
static const int asn_MAP_NodeAttributeSet_addGrpC_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_NodeAttributeSet_addGrpC_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_NodeAttributeSet_addGrpC_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* ptvRequest */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* nodeLink */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* node */
};
asn_SEQUENCE_specifics_t asn_SPC_NodeAttributeSet_addGrpC_specs_1 = {
	sizeof(struct NodeAttributeSet_addGrpC),
	offsetof(struct NodeAttributeSet_addGrpC, _asn_ctx),
	asn_MAP_NodeAttributeSet_addGrpC_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_NodeAttributeSet_addGrpC_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_NodeAttributeSet_addGrpC = {
	"NodeAttributeSet-addGrpC",
	"NodeAttributeSet-addGrpC",
	&asn_OP_SEQUENCE,
	asn_DEF_NodeAttributeSet_addGrpC_tags_1,
	sizeof(asn_DEF_NodeAttributeSet_addGrpC_tags_1)
		/sizeof(asn_DEF_NodeAttributeSet_addGrpC_tags_1[0]), /* 1 */
	asn_DEF_NodeAttributeSet_addGrpC_tags_1,	/* Same as above */
	sizeof(asn_DEF_NodeAttributeSet_addGrpC_tags_1)
		/sizeof(asn_DEF_NodeAttributeSet_addGrpC_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_NodeAttributeSet_addGrpC_1,
	3,	/* Elements count */
	&asn_SPC_NodeAttributeSet_addGrpC_specs_1	/* Additional specs */
};

