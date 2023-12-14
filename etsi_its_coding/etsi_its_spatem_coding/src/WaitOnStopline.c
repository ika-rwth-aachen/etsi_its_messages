/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_spatem_coding/WaitOnStopline.h"

/*
 * This type is implemented using BOOLEAN,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_WaitOnStopline_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (1 << 2))
};
asn_TYPE_descriptor_t asn_DEF_WaitOnStopline = {
	"WaitOnStopline",
	"WaitOnStopline",
	&asn_OP_BOOLEAN,
	asn_DEF_WaitOnStopline_tags_1,
	sizeof(asn_DEF_WaitOnStopline_tags_1)
		/sizeof(asn_DEF_WaitOnStopline_tags_1[0]), /* 1 */
	asn_DEF_WaitOnStopline_tags_1,	/* Same as above */
	sizeof(asn_DEF_WaitOnStopline_tags_1)
		/sizeof(asn_DEF_WaitOnStopline_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		BOOLEAN_constraint
	},
	0, 0,	/* No members */
	0	/* No specifics */
};

