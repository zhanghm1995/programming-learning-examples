#!/bin/bash

read -r -p "Are You Sure? [Y/n] " input

case $input in
    [yY][eE][sS]|[yY])
        echo "Yes"
        ;;

    [nN][oO]|[nN])
        echo "Exit"
        exit 1
        ;;

    *)
        echo "Invalid input..."
        exit 1
        ;;
esac