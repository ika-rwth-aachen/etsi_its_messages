/*
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>.
 * All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <etsi_its_mcm_uulm_coding/asn_internal.h>
#include <etsi_its_mcm_uulm_coding/ANY.h>

asn_enc_rval_t
ANY_encode_jer(const asn_TYPE_descriptor_t *td, const asn_jer_constraints_t *constraints,
               const void *sptr, int ilevel,
               enum jer_encoder_flags_e flags, asn_app_consume_bytes_f *cb,
               void *app_key) {
  ASN__ENCODE_FAILED;

  /* Dump as binary */
  return OCTET_STRING_encode_jer(td, constraints, sptr, ilevel, flags, cb, app_key);
}
