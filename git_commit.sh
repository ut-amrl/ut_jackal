#!/bin/bash
set -x -e
if [ -z "$1" ]
then
	echo "ERROR: Must specify username, e.g. \"joydeepb@cs.utexas.edu\""
	exit 1
fi
USERNAME=$1
git commit --author "$USERNAME" 
