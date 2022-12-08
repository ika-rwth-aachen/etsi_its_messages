# ------------------------------------------------------------------------------
# ReservationRequestMessage ::= SEQUENCE {
#  currentTime			TimestampUTC,
#  preReservation-ID	PreReservation-ID,
#  arrivalTime			TimestampUTC,
#  departureTime		TimestampUTC OPTIONAL,
#  eAmount				EAmount,
#  eAmountMin			EAmount,
#  paymentType			PaymentType,
#  payment-ID 			Payment-ID,
#  secondPayment-ID	Payment-ID OPTIONAL,
#  pairing-ID			Pairing-ID OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampUTC currentTime

PreReservation_ID preReservation_ID

TimestampUTC arrivalTime

TimestampUTC departureTime
bool departureTime_isPresent

EAmount eAmount

EAmount eAmountMin

PaymentType paymentType

Payment_ID payment_ID

Payment_ID secondPayment_ID
bool secondPayment_ID_isPresent

Pairing_ID pairing_ID
bool pairing_ID_isPresent
