#!/bin/bash
#
# Copyright (c) 2023 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#

# Deployment automatisation procedure to build all of the librairies of the Smart Data Engine related packages
# Deprecated script. Use CLI instead i.e heex sdk build --help
# Keep the arguments availaible for backward compatibility
set -e  # Stop if one command fails

#
# Help & Usage
#
print_usage()
{
  ./heex sdk build --help
  echo "Usage:"
  echo "     '--build-sdk' (-f) to build the sdk (default: true)"
  echo "     '--build-examples' (-e) to build available examples."
  echo "     '--with-python' (-p) to build the libraries with the python flags"
  echo "     '--with-legacy-cmake' (-c) to only build with older cmake commands"
  echo "     '--with-legacy-ubuntu18' (-l) to build for 18.04 with older libs and cmake commands."
  echo "     '--with-legacy-getrandom' (-m) to force the build with the get random fix. Add flag BOOST_UUID_RANDOM_PROVIDER_FORCE_POSIX. Is added per default on Linux Kernel <= 3.17."
  echo "     '--with-boost-version' (-b) to specify a specific version of Boost to build the SDK with."
  echo "     '--verbose' (-i) to obtain build info during and once the build is completed."
  echo "     '--rebuild' (-r) to build from scratch every package of the SDK."
  echo "     '--jobs N' (-j) to allow building with N jobs at once; infinite jobs with no arg but requires to be last."
  echo "     '--debug' (-d) to only build with the Debug flag."
  echo "     '--cleanDependencies' (-g) to remove all dependencies managed by the Heex SDK within the 3rdparty folder."
}

#
# Input options
#
# Transform long options to short ones
for arg in "$@"; do
  shift
  case "$arg" in
    '--help')                 set -- "$@" '-h'   ;;
    '--with-python')          set -- "$@" '-p'   ;;
    '--no-auto')              set -- "$@" '-a'   ;;
    '--with-legacy-ubuntu18') set -- "$@" '-l'   ;;
    '--with-legacy-cmake')    set -- "$@" '-c'   ;;
    '--debug')                set -- "$@" '-d'   ;;
    '--build-sdk')            set -- "$@" '-f'   ;;
    '--verbose')              set -- "$@" '-i'   ;;
    '--rebuild')              set -- "$@" '-r'   ;;
    '--build-examples')       set -- "$@" '-e'   ;;
    '--with-legacy-getrandom')set -- "$@" '-m'   ;;
    '--cleanDependencies')    set -- "$@" '-g'   ;;
    '--with-boost-version')   set -- "$@" '-b'   ;;
    '--jobs')                 set -- "$@" '-j'   ;;
    *)                        set -- "$@" "$arg" ;;
  esac
done

# Default behavior
cmdHeexCli="./heex sdk build"                     # Heex CLI command.
defaultBoost=1.75.0
cmdHeexCliArgs=                                   # All arguments for heex cli commnand
buildBoostVersion=$defaultBoost                   # Determine the version of Boost libraries to use. Set to $defaultBoost by default.
buildJobsNb=-1                                    # Determine number of parrallel jobs to build with cmake. To the max available on the machine when empty. Set to -1 by default.
cleanDependenciesOnly="false"                     # Determine if all dependencies managed by the Heex SDK within the 3rdparty folder shall be removed and return once completed.

# Parse short options
OPTIND=1
while getopts ":hpalcdfstuirekmnogb::j::" opt
do
  case "$opt" in
    'h') print_usage; exit 0 ;;
    'p') cmdHeexCliArgs="$cmdHeexCliArgs --with-python" ;;
    'a') echo "[WARNING] Bootstrap option -a has been depreciated and is not needed anymore. For compatibility, the Sdk will still be built." >&2 ;;
    'l') cmdHeexCliArgs="$cmdHeexCliArgs --with-legacy-ubuntu18" ;;
    'c') cmdHeexCliArgs="$cmdHeexCliArgs --with-legacy-cmake" ;;
    'd') cmdHeexCliArgs="$cmdHeexCliArgs --debug" ;;
    'f') ;;
    'i') cmdHeexCliArgs="$cmdHeexCliArgs --verbose" ;;
    'r') cmdHeexCliArgs="$cmdHeexCliArgs --rebuild" ;;
    'e') cmdHeexCliArgs="$cmdHeexCliArgs --build-examples" ;;
    'm') cmdHeexCliArgs="$cmdHeexCliArgs --with-legacy-getrandom" ;;
    'n') ;;
    'g') cleanDependenciesOnly="true" ;;
    'b') buildBoostVersion="$OPTARG";;
    'j') buildJobsNb="$OPTARG"; echo "buildJobsNb=${buildJobsNb}";;
    ':') if [[ $OPTARG == "j" ]]; then buildJobsNb=-1; fi
         if [[ $OPTARG == "b" ]]; then buildBoostVersion=$defaultBoost; fi ;;
    '?') echo "Unknown argument: -$OPTARG"; print_usage >&2; exit 1 ;;
  esac
done
shift $(expr $OPTIND - 1) # remove options from positional parameters


if [[ ! -x heex ]]
then
  echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk --help\e[0m"
  echo "The CLI executable 'heex' can't be found."
  echo "Check your package"
  exit 1
fi

# Mapping to CLI clean
if [[ "$cleanDependenciesOnly" == "true" ]]
then
  echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk clean --help\e[0m"
  ./heex sdk clean
  if [[ "$?" != "0" ]]; then
    echo "Sdk clean failed. Check logs"
    echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk clean --help\e[0m"
    exit 1
  fi
  echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk clean --help\e[0m"
  exit 0
fi

echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk build --help\e[0m"

if [[ "$buildBoostVersion" != "$defaultBoost" ]]
then
  cmdHeexCliArgs="$cmdHeexCliArgs --with-boost-version $buildBoostVersion"
fi

if [[ $buildJobsNb != -1 ]]
then
  cmdHeexCliArgs="$cmdHeexCliArgs --jobs $buildJobsNb"
fi

echo "Heex CLI build args : $cmdHeexCliArgs"

eval "$cmdHeexCli $cmdHeexCliArgs"
if [[ "$?" != "0" ]]; then
  echo "Sdk build failed. Check logs"
  echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk build --help\e[0m"
  exit 1
fi
echo -e "\e[31mThis script is deprecated. Please consider using the Heex CLI instead : ./heex sdk build --help\e[0m"

exit 0
