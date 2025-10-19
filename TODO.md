- [ ] fixes needed for GddAttribute.msg, GddStructure.msg, ISO14823Code.msg (see TODOs in msg files)


---------------------------------------------------
Further modifications required for generating msgs
---------------------------------------------------

/*
asn1/raw/is_ts103301/build/asn1/ISO19321IVIv2.asn
TODO: changed from Temperature to Temperature2
*/
Temperature2::=INTEGER (-100..151)

---------------------------------------------------

/*
asn1/raw/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn
TODO: changed from version to version1
*/
version1 INTEGER(0..255)::= 1 -- version of this ASN.1 module

---------------------------------------------------

diff --git a/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn b/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn
index 49049c5..98f054c 100644
--- a/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn
+++ b/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn
@@ -210,7 +210,7 @@ unitType UnitType,
 value INTEGER (0..32767)
 },
 absorptionCoeff Int2 }
-DriverCharacteristics ::= SEQUENCE {
+DriverCharacteristics2 ::= SEQUENCE {
 driverClass Int1,
 tripPurpose Int1
 }
@@ -461,7 +461,7 @@ sessionTime DateAndTime,
 sessionServiceProvider Provider,
 stationLocation INTEGER(0..1048575),
 sessionLocation BIT STRING (SIZE(8)),
-typeOfSession StationType,
+typeOfSession StationType2,
 sessionResultOperational ResultOp,
 sessionResultFinancial ResultFin
 }
@@ -528,7 +528,7 @@ SessionLocation ::= SEQUENCE {
 ascendingKilometrage BOOLEAN, -- travel direction indicator
 laneCodeNumber INTEGER(0..127) -- lane code number
 }
-StationType ::= ENUMERATED {
+StationType2 ::= ENUMERATED {
 unspecified (0),
 closedEntryWithPayment (1),
 closedEntryWithoutPayment (2),
@@ -563,7 +563,7 @@ SoundLevel ::= SEQUENCE{
 soundstationary Int1,
 sounddriveby Int1
 }
-TrailerCharacteristics ::= SEQUENCE {
+TrailerCharacteristics2 ::= SEQUENCE {
 trailerDetails TrailerDetails,
 trailerMaxLadenWeight Int2,
 trailerWeightUnladen Int2