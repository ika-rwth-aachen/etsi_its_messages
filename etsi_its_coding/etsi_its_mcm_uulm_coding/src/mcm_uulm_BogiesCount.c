/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=mcm_uulm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_mcm_uulm_coding/mcm_uulm_BogiesCount.h"

int
mcm_uulm_BogiesCount_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 2L && value <= 101L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_mcm_uulm_BogiesCount_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 7,  7,  2,  101 }	/* (2..101) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const ber_tlv_tag_t asn_DEF_mcm_uulm_BogiesCount_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
asn_TYPE_descriptor_t asn_DEF_mcm_uulm_BogiesCount = {
	"BogiesCount",
	"BogiesCount",
	&asn_OP_NativeInteger,
	asn_DEF_mcm_uulm_BogiesCount_tags_1,
	sizeof(asn_DEF_mcm_uulm_BogiesCount_tags_1)
		/sizeof(asn_DEF_mcm_uulm_BogiesCount_tags_1[0]), /* 1 */
	asn_DEF_mcm_uulm_BogiesCount_tags_1,	/* Same as above */
	sizeof(asn_DEF_mcm_uulm_BogiesCount_tags_1)
		/sizeof(asn_DEF_mcm_uulm_BogiesCount_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_mcm_uulm_BogiesCount_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		mcm_uulm_BogiesCount_constraint
	},
	0, 0,	/* Defined elsewhere */
	0	/* No specifics */
};

