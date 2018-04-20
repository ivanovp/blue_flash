#!/bin/sh
FNAME=test.bin
FNAME2=test.bin.upload
DUMP1=$FNAME.dump
DUMP2=$FNAME2.dump
SIZE=`dfu-util -l|grep Size|perl -p -e 's/.*Size: (\d+) bytes.*/\1/'`
BS=65536
COUNT=$(expr $SIZE / $BS)
echo Generating $SIZE bytes random file...
dd if=/dev/urandom of=$FNAME count=$COUNT bs=$BS
echo Done.
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
