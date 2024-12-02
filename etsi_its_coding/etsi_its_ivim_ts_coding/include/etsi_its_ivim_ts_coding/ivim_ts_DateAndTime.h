/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_DateAndTime_H_
#define	_ivim_ts_DateAndTime_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_DateCompact.h"
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_DateAndTime */
typedef struct ivim_ts_DateAndTime {
	ivim_ts_DateCompact_t	 timeDate;
	struct ivim_ts_DateAndTime__timeCompact {
		long	 hours;
		long	 mins;
		long	 double_secs;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} timeCompact;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_DateAndTime_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_DateAndTime;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_DateAndTime_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_DateAndTime_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_DateAndTime_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
