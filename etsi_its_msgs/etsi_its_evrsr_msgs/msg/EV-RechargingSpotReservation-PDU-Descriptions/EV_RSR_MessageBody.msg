# ------------------------------------------------------------------------------
# EV-RSR-MessageBody ::= CHOICE {
#  preReservationRequestMessage		PreReservationRequestMessage,
#  preReservationResponseMessage		PreReservationResponseMessage,
#  reservationRequestMessage				ReservationRequestMessage,
#  reservationResponseMessage			ReservationResponseMessage,
#  cancellationRequestMessage			CancellationRequestMessage,
#  cancellationResponseMessage			CancellationResponseMessage,
#  updateRequestMessage						UpdateRequestMessage,
#  updateResponseMessage					UpdateResponseMessage,
# ...
# }
# ------------------------------------------------------------------------------

uint8 choice

PreReservationRequestMessage preReservationRequestMessage
uint8 CHOICE_PRE_RESERVATION_REQUEST_MESSAGE = 0
PreReservationResponseMessage preReservationResponseMessage
uint8 CHOICE_PRE_RESERVATION_RESPONSE_MESSAGE = 1
ReservationRequestMessage reservationRequestMessage
uint8 CHOICE_RESERVATION_REQUEST_MESSAGE = 2
ReservationResponseMessage reservationResponseMessage
uint8 CHOICE_RESERVATION_RESPONSE_MESSAGE = 3
CancellationRequestMessage cancellationRequestMessage
uint8 CHOICE_CANCELLATION_REQUEST_MESSAGE = 4
CancellationResponseMessage cancellationResponseMessage
uint8 CHOICE_CANCELLATION_RESPONSE_MESSAGE = 5
UpdateRequestMessage updateRequestMessage
uint8 CHOICE_UPDATE_REQUEST_MESSAGE = 6
UpdateResponseMessage updateResponseMessage
uint8 CHOICE_UPDATE_RESPONSE_MESSAGE = 7
