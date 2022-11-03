# ------------------------------------------------------------------------------
# PreReservationResponseMessage ::= SEQUENCE {
#  preReservation-ID						PreReservation-ID,
#  availabilityStatus					AvailabilityStatus,
#  preReservationExpirationTime	TimestampUTC,
#  supportedPaymentTypes 				SupportedPaymentTypes,
#  ...
# }
# ------------------------------------------------------------------------------

PreReservation_ID preReservation_ID

AvailabilityStatus availabilityStatus

TimestampUTC preReservationExpirationTime

SupportedPaymentTypes supportedPaymentTypes
