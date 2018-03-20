#!/bin/sh
FNAME=$1
FNAME2=$1.upload
DUMP1=$FNAME.dump
DUMP2=$FNAME2.dump
if [ "$1" != "" ]; then
    rm -f $FNAME2 $DUMP1 $DUMP2
    echo "Downloading $FNAME..."
    dfu-util -s 0 -D $FNAME
    echo "Uploading $FNAME2..."
    dfu-util -s 0 -U $FNAME2
    diff $FNAME $FNAME2
    rc=$?; 
    if [[ $rc != 0 ]]; then 
        echo "TEST FAILED!"
        xxd $FNAME >$DUMP1
        xxd $FNAME2 >$DUMP2
        gvimdiff $DUMP1 $DUMP2
    else
        echo "TEST PASSED!"
        rm $FNAME2
    fi
else
    echo "Missing parameter!"
    echo
    echo "Usage: $0 <filename>"
fi
