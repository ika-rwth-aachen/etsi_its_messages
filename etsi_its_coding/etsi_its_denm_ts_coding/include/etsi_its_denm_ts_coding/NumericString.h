/*-
 * Copyright (c) 2003-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_NumericString_H_
#define	_NumericString_H_

#include <etsi_its_denm_ts_coding/OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef OCTET_STRING_t NumericString_t;	/* Implemented via OCTET STRING */

extern asn_TYPE_descriptor_t asn_DEF_NumericString;
extern asn_TYPE_operation_t asn_OP_NumericString;

#define NumericString_free OCTET_STRING_free

#if !defined(ASN_DISABLE_PRINT_SUPPORT)
#define NumericString_print OCTET_STRING_print_utf8
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) */

#define NumericString_compare OCTET_STRING_compare
#define NumericString_copy    OCTET_STRING_copy

asn_constr_check_f NumericString_constraint;

#if !defined(ASN_DISABLE_BER_SUPPORT)
#define NumericString_decode_ber OCTET_STRING_decode_ber
#define NumericString_encode_der OCTET_STRING_encode_der
#endif  /* !defined(ASN_DISABLE_BER_SUPPORT) */

#if !defined(ASN_DISABLE_XER_SUPPORT)
#define NumericString_decode_xer OCTET_STRING_decode_xer_utf8
#define NumericString_encode_xer OCTET_STRING_encode_xer_utf8
#endif  /* !defined(ASN_DISABLE_XER_SUPPORT) */

#if !defined(ASN_DISABLE_JER_SUPPORT)
#define NumericString_decode_jer OCTET_STRING_decode_jer_utf8
#define NumericString_encode_jer OCTET_STRING_encode_jer_utf8
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */

#if !defined(ASN_DISABLE_OER_SUPPORT)
#define NumericString_decode_oer OCTET_STRING_decode_oer
#define NumericString_encode_oer OCTET_STRING_encode_oer
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */

#if !defined(ASN_DISABLE_UPER_SUPPORT)
#define NumericString_decode_uper OCTET_STRING_decode_uper
#define NumericString_encode_uper OCTET_STRING_encode_uper
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) */
#if !defined(ASN_DISABLE_APER_SUPPORT)
#define NumericString_decode_aper OCTET_STRING_decode_aper
#define NumericString_encode_aper OCTET_STRING_encode_aper
#endif  /* !defined(ASN_DISABLE_APER_SUPPORT) */

#if !defined(ASN_DISABLE_RFILL_SUPPORT)
#define NumericString_random_fill OCTET_STRING_random_fill
#endif  /* !defined(ASN_DISABLE_RFILL_SUPPORT) */

#ifdef __cplusplus
}
#endif

#endif	/* _NumericString_H_ */
