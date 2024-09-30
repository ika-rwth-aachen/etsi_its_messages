/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_mapem_ts_coding/WMInumber.h"
#include "etsi_its_mapem_ts_coding/VDS.h"
#include <etsi_its_mapem_ts_coding/constr_SEQUENCE.h>
#ifndef	_VehicleIdentification_H_
#define	_VehicleIdentification_H_


#include <etsi_its_mapem_ts_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VehicleIdentification */
typedef struct VehicleIdentification {
	WMInumber_t	*wMInumber;	/* OPTIONAL */
	VDS_t	*vDS;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleIdentification_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleIdentification;

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleIdentification_H_ */
#include <etsi_its_mapem_ts_coding/asn_internal.h>
