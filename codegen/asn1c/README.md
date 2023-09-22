## C++ compatible C source code for ETSI ITS messages generated from ASN.1 using asn1c

### Example Usage

```bash
# etsi_its_messages/ $

docker run \
    -v $(pwd)/../../asn1/reduced/cam:/input:ro \
    -v $(pwd)/../../etsi_its_coding/etsi_its_cam_coding:/output \
    -u $(id -u):$(id -g) \
        gitlab.ika.rwth-aachen.de:5050/fb-fi/definitions/etsi_its_messages/asn1c:latest

./postprocessAsn1cIncludes.py etsi_its_coding/etsi_its_cam_coding
```
