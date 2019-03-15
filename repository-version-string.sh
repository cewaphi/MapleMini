#!/bin/sh

# this script will store a current, unique descriptor for the bitbake git repository
# and save it in a way that bitbake can read it and e.g. append it to the distro version.

HASH=`git rev-parse --short HEAD`
if [ "x`git status -s`" = "x" ]; then
	DIRTY=""
else
	DIRTY="-dirty"
fi
if [ "$1" = "usbversion" ]; then
	echo "$HASH$DIRTY" | sed -re "s/(.)/'\\1',0,/g"
elif [ "$1" = "length" ]; then
	printf "$HASH$DIRTY" | wc -c
else
	echo "$HASH$DIRTY"
fi


