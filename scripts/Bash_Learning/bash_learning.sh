#!/bin/bash

echo "Hello world" "Hello world"
echo $0

echo "Start uploading $1..."


SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
echo $SHELL_FOLDER