#!/bin/bash
 
#Declare a string array
LanguageArray=("PHP"  "Java"  "C#"  "C++"  "VB.Net"  "Python"  "Perl")
 
# Print array values in  lines
echo "Print every element in new line"
for val1 in ${LanguageArray[*]}; do
     echo $val1
done
 
echo ""
 
# Print array values in one line
echo "Print all elements in a single line"
for val2 in "${LanguageArray[*]}"; do
    echo $val2
done
echo ""