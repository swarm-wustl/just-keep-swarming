# Make sure this is run in its parent directory (i.e., esp)
# ex: sh scripts/[name].sh
. $IDF_PATH/export.sh
idf.py build
idf.py flash
idf.py monitor