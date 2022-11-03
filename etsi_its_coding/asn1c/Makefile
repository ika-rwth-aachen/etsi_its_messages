# === VARIABLES ===================================================================
REGISTRY = gitlab.ika.rwth-aachen.de:5050
IMAGE_BASE = automated-driving/ros_etsi_its_messages
IMAGE_ASN1C = asn1c
IMAGE_TAG = latest

# === HELPERS ==================================================================

all: help

help:
	@echo "Available make targets:"
	@echo "==========================================="
	@fgrep -h '##' $(MAKEFILE_LIST) | fgrep -v fgrep | sed 's/\\$$//' | sed 's/##//'
	@echo "-------------------------------------------"
	@echo "pull-asn1c:        pull asn1c image"
	@echo "build-asn1c:       build asn1c image"
	@echo "compile-asn1:      compile asn1-files stored within the 'input'-folder, the generated *.c/*.h files will be stored in the 'output'-folder"


login:		## login to GitLab registry
	@echo "Please sign in at the GitLab server using your ika LDAP account."
	@docker login $(REGISTRY)

# === BUILD ====================================================================

build-asn1c:
	@cd docker && \
		docker build \
		--tag $(REGISTRY)/$(IMAGE_BASE)/$(IMAGE_ASN1C):$(IMAGE_TAG) .

# === PULL =====================================================================
pull-asn1c: login
	@docker pull $(REGISTRY)/$(IMAGE_BASE)/$(IMAGE_ASN1C):$(IMAGE_TAG)

# === ASN1 CODING =====================================================================
compile-asn1:
	@docker run --volume /home/gkueppers/git/ros_etsi_its_messages/etsi_its_coding/asn1c/input:/home/input --volume /home/gkueppers/git/ros_etsi_its_messages/etsi_its_coding/asn1c/output:/home/output $(REGISTRY)/$(IMAGE_BASE)/$(IMAGE_ASN1C):$(IMAGE_TAG)
