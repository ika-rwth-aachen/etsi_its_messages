#!/bin/bash
asn_files=()

function loop_subfolders {
    asn_files+=($(pwd)/*.asn)
    for f in *; do
        if [ -d "$f" ]; then
            cd "$f"
            loop_subfolders
            cd ..
        fi
    done
}

cd input
shopt -s nullglob
loop_subfolders
unique_asn_files=()
for file in ${asn_files[@]}; do
    bn=$(basename "${file}")
    echo $bn
    not_in_list=True
    for un in ${unique_asn_files[@]}; do
        ubn=$(basename "${un}")
        echo $ubn
       if [ubn==bn]; then
            not_in_list=False
       fi
    done
    if [${not_in_list}]; then
        unique_asn_files+=(${file})
    fi
done
echo ${unique_asn_files[@]}
#asn1c ${asn_files[@]} -fcompound-names -no-gen-example -gen-PER
#chmod a+rw *
#mkdir ../output/src ../output/include
#chmod a+rw ../output/src ../output/include
#mv *.c ../output/src
#mv *.h ../output/include
#rm *.libasncodec

