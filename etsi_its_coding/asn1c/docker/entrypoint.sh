#!/bin/bash
cd input
shopt -s nullglob
asn_files=(*.asn)
asn1c ${asn_files[@]} -fcompound-names -no-gen-example -gen-PER
chmod a+rw *
mkdir ../output/src ../output/include
chmod a+rw ../output/src ../output/include
mv *.c ../output/src
mv *.h ../output/include
rm *.libasncodec

