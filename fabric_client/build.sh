#!/bin/bash

path=`pwd | xargs dirname`
make KBUILD_EXTRA_SYMBOLS=$path/Module.symvers
