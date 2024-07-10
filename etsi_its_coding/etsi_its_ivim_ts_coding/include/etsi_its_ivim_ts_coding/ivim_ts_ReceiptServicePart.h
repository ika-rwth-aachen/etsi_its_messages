/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_ReceiptServicePart_H_
#define	_ivim_ts_ReceiptServicePart_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_DateAndTime.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Provider.h"
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include <etsi_its_ivim_ts_coding/BIT_STRING.h>
#include "etsi_its_ivim_ts_coding/ivim_ts_EfcDsrcApplication_StationType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ResultOp.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ResultFin.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_ReceiptServicePart */
typedef struct ivim_ts_ReceiptServicePart {
	ivim_ts_DateAndTime_t	 sessionTime;
	ivim_ts_Provider_t	 sessionServiceProvider;
	long	 stationLocation;
	BIT_STRING_t	 sessionLocation;
	ivim_ts_EfcDsrcApplication_StationType_t	 typeOfSession;
	ivim_ts_ResultOp_t	 sessionResultOperational;
	ivim_ts_ResultFin_t	 sessionResultFinancial;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_ReceiptServicePart_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_ReceiptServicePart;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_ReceiptServicePart_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
