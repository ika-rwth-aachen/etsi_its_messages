/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_ReceiptData_H_
#define	_ivim_ts_ReceiptData_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_DateAndTime.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Provider.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Int2.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_SessionLocation.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Int1.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ResultOp.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_PaymentFee.h"
#include <etsi_its_ivim_ts_coding/OCTET_STRING.h>
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_ReceiptData */
typedef struct ivim_ts_ReceiptData {
	ivim_ts_DateAndTime_t	 sessionTime;
	ivim_ts_Provider_t	 sessionServiceProvider;
	ivim_ts_Int2_t	 locationOfStation;
	ivim_ts_SessionLocation_t	 sessionLocation;
	ivim_ts_Int1_t	 sessionType;
	ivim_ts_ResultOp_t	 sessionResult;
	ivim_ts_Int1_t	 sessionTariffClass;
	ivim_ts_Int1_t	 sessionClaimedClass;
	ivim_ts_PaymentFee_t	 sessionFee;
	ivim_ts_Provider_t	 sessionContractProvider;
	OCTET_STRING_t	 sessionTypeOfContract;
	long	 sessionContextVersion;
	OCTET_STRING_t	 receiptDataAuthenticator;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_ReceiptData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_ReceiptData;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_ReceiptData_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_ReceiptData_1[13];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_ReceiptData_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
