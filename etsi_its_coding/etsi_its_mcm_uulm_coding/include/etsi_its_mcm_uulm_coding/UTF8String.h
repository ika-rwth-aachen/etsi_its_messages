/*-
 * Copyright (c) 2003-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_UTF8String_H_
#define	_UTF8String_H_

#include <etsi_its_mcm_uulm_coding/OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef OCTET_STRING_t UTF8String_t;	/* Implemented via OCTET STRING */

extern asn_TYPE_descriptor_t asn_DEF_UTF8String;
extern asn_TYPE_operation_t asn_OP_UTF8String;

#define UTF8String_free OCTET_STRING_free

#if !defined(ASN_DISABLE_PRINT_SUPPORT)
asn_struct_print_f UTF8String_print;
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) */

#define UTF8String_compare OCTET_STRING_compare
#define UTF8String_copy    OCTET_STRING_copy

asn_constr_check_f UTF8String_constraint;

#if !defined(ASN_DISABLE_BER_SUPPORT)
#define UTF8String_decode_ber OCTET_STRING_decode_ber
#define UTF8String_encode_der OCTET_STRING_encode_der
#endif  /* !defined(ASN_DISABLE_BER_SUPPORT) */

#if !defined(ASN_DISABLE_XER_SUPPORT)
#define UTF8String_decode_xer OCTET_STRING_decode_xer_utf8
#define UTF8String_encode_xer OCTET_STRING_encode_xer_utf8
#endif  /* !defined(ASN_DISABLE_XER_SUPPORT) */

#if !defined(ASN_DISABLE_JER_SUPPORT)
#define UTF8String_decode_jer OCTET_STRING_decode_jer_utf8
#define UTF8String_encode_jer OCTET_STRING_encode_jer_utf8
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */

#if !defined(ASN_DISABLE_UPER_SUPPORT)
#define UTF8String_decode_uper OCTET_STRING_decode_uper
#define UTF8String_encode_uper OCTET_STRING_encode_uper
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) */
#if !defined(ASN_DISABLE_APER_SUPPORT)
#define UTF8String_decode_aper OCTET_STRING_decode_aper
#define UTF8String_encode_aper OCTET_STRING_encode_aper
#endif  /* !defined(ASN_DISABLE_APER_SUPPORT) */

#if !defined(ASN_DISABLE_RFILL_SUPPORT)
asn_random_fill_f UTF8String_random_fill;
#endif  /* !defined(ASN_DISABLE_RFILL_SUPPORT) */

/*
 * Returns length of the given UTF-8 string in characters,
 * or a negative error code:
 * -1:	UTF-8 sequence truncated 
 * -2:	Illegal UTF-8 sequence start
 * -3:	Continuation expectation failed
 * -4:	Not minimal length encoding
 * -5:	Invalid arguments
 */
ssize_t UTF8String_length(const UTF8String_t *st);

/*
 * Convert the UTF-8 string into a sequence of wide characters.
 * Returns the number of characters necessary.
 * Returned value might be greater than dstlen.
 * In case of conversion error, 0 is returned.
 * 
 * If st points to a valid UTF-8 string, calling
 * 	UTF8String_to_wcs(st, 0, 0);
 * is equivalent to
 * 	UTF8String_length(const UTF8String_t *st);
 */
size_t UTF8String_to_wcs(const UTF8String_t *st, uint32_t *dst, size_t dstlen);

#ifdef __cplusplus
}
#endif

#endif	/* _UTF8String_H_ */
