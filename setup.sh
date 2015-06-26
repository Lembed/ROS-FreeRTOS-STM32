#!/bin/bash

echo "Setting up CodeSourcery and st-link in $1..."
DIR=$1
if [ -d "$DIR" ]; then
	# Control will enter here if $DIRECTORY exists.
	if mkdir -p "$DIR/CodeSourcery" -v ; then
    	# success
    	cd $DIR/CodeSourcery
		wget https://sourcery.mentor.com/sgpp/lite/arm/portal/package9740/public/arm-none-eabi/arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
		tar xvf arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
		rm arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
		echo PATH=\"\$PATH:$PWD/arm-2011.09/bin\" >> ~/.bashrc
		echo "Extracted CodeSourcery contents. Binaries are located in $PWD/arm-2011.09/bin ."
		
		cd ..
		git clone git://github.com/texane/stlink.git
		cd stlink
		./autogen.sh
		./configure
		make
		echo PATH=\"\$PATH:$PWD\" >> ~/.bashrc
		echo "Extracted st-link contents. Binaries are located in $PWD ."
		echo "Setup was successfully completed."
		echo "Type source ~/.bashrc for environment variables to get effective."
	else
		# failure
	    echo "$DIR/CodeSourcery cannot be created. Aborting setup."
	fi
else
	# Control will enter here if $DIRECTORY doesn't exist.
	echo "Directory $DIR does not exist."
	echo "Usage: ./setup.sh [YOUR_SETUP_DIRECTORY]"
fi


