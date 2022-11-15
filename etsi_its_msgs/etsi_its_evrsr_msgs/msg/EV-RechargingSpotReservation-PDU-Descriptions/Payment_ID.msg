# ------------------------------------------------------------------------------
# Payment-ID ::= CHOICE {
#  contractID ContractID,
#  externalIdentificationMeans	ExternalIdentificationMeans
# }
# ------------------------------------------------------------------------------

uint8 choice

ContractID contractID
uint8 CHOICE_CONTRACT_I_D = 0
ExternalIdentificationMeans externalIdentificationMeans
uint8 CHOICE_EXTERNAL_IDENTIFICATION_MEANS = 1
