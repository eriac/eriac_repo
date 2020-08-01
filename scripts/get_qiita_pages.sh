#!bin/bash/

wget https://qiita.com/srs/items/5f44440afea0eb616b4a.md -O index.md

COUNTER=0
PDF_FILES=""
while read line
do
  URL=`echo $line | grep -zoP -- 'http(s?)://[0-9a-zA-Z?=#+_&:/.%\n]+'`
  if [ -n "$URL" ]; then
    echo $URL
    FILE_NAME=`basename $URL.md`
    if [ ! -f $FILE_NAME ]; then
      wget $URL.md
    fi
    TITLE=`cat $FILE_NAME | sed -n 2P | cut -c 8-| sed "s/ /_/g"`
    if [ ! -f $FILE_NAME.pdf ]; then
      pandoc -V documentclass=ltjarticle -V geometry:margin=20mm --latex-engine=lualatex $FILE_NAME -o $FILE_NAME.pdf
    fi
    PDF_FILES=$PDF_FILES" "$FILE_NAME.pdf
    COUNTER=$((COUNTER+1))
    echo $COUNTER
    #if [ $COUNTER -eq 3 ]; then
    #  echo break
    #  break
    #fi
  fi
done < ./index.md

echo $PDF_FILES
pdfunite $PDF_FILES all.pdf
