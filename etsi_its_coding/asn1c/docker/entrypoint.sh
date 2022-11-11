#!/bin/bash
asn_files=()

# Function to find all asn files in the current directory and all subdirectories
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

# Get all ASN-Files in given input directory
cd input
shopt -s nullglob
loop_subfolders

# Remove ASN1-File duplicates from list
unique_asn_files=()
for file in ${asn_files[@]}; do
    bn=$(basename "${file}")
    not_in_list=True
    for un in ${unique_asn_files[@]}; do
        ubn=$(basename "${un}")
       if [ "$ubn" == "$bn" ]; then
            echo "WARNING: ${un} is already in list! ${file} will not be added to the list! Make sure, that you're revisions are matching, otherwise the compilation could fail!"
            not_in_list=False
       fi
    done
    if [ ${not_in_list} = True ]; then
        unique_asn_files+=(${file})
    fi
done

# Compile the ASN1 Files
asn1c ${unique_asn_files[@]} -fcompound-names -no-gen-example -gen-PER

# Move resulting output to the output folder and clean-up
chmod a+rw *
mkdir ../output/src ../output/include
chmod a+rw ../output/src ../output/include
mv *.c ../output/src
mv *.h ../output/include
rm *.libasncodec