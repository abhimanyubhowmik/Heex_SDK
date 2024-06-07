#!/bin/bash
#
# Copyright (c) 2023 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#

# Launch automatisation procedure to run all of the Heex executables of the Smart Data Engine (forming Heex Kernel) and additional agents

#
# Default variables
#
defaultConfigFilepath="./kernel/systemConfs/systemConf.json"  # Set the default filepath for the Heex SDK configuration file relative to the Heex SDK folder
oldDefaultConfigFilepath="./kernel/systemConf.json"  # Set the default filepath for the Heex SDK configuration file relative to the Heex SDK folder
configFilepath=""         # Cache the value to the ConfigFilepath default value
runKill="false"           # Determine if the kernel process should be killed
noUpload="false"          # Determine if the kernel process should be launched without the data uploader
metadataOnlyMode="false"  # Cache the value to the ConfigFilepath default value
uploadOnlyMode="false"    # Determine if the upload only mode is set
runAsService="false"      # Determine if this script should run without exiting
previousDir=              # Path to cache if the folder path before the cd command get performed to the Heex SDK folder
heexDataUploaderArgs=     # Cache any extra args to provide to the heexDataUploader process
heexDefaultWorkingDirectory="$HOME/.heex" # Working directory to store system data, logs and caches.
promptAnswerIsYes="false"         # Yes to all prompts
waitForJobsCompletion="false" # Wait for jobs completion default status
heexDatabaseFile="HeexJobs.db"

#
# Help & Usage
#
print_usage()
{
  echo "Usage:"
  echo "     '--help' (-h) to get usage and additional info."
  echo "     '--config' (-c) to run the Kernel with the specified configuration. Default: ${defaultSystemConfPath}."
  echo "     '--no-upload' (-n) to run the Kernel without the data uploader."
  echo "     '--run-as-service' (-s) to run the Heex Kernel script without exiting."
  echo "     '--metadata-only' (-m) to enable the metadata-only mode that only uploads metadata to Heex APIs.
          Default is false. Non-uploaded event recordings are kept listed in the 'heexSmartDataUploadJobs.json' file.
          Rerun this script without this flag to perform the upload of stalled event recordings."
  echo "     '--upload-only' (-u) to run the Data Uploader only. It will upload all recordings and metadata stored
          in the 'recordingsFolderPath' set in the 'kernel/DataSender.conf' file."
  echo "     '--kill' (-k) to kill all processes involved in the Heex Kernel. Does not run the kernel."
  echo "     '--wait-for-jobs-completion (-w) to wait for jobs completion if --kill option is specified."
  echo "     '--yes' (-y) to accept all prompts."
}

function promptUserForChoice ()
{
  local message=$1
  echo "$message [Y/n]"

  if [[ "$promptAnswerIsYes" == "true" ]]; then
    echo "Autoreply Yes"
    return 0
  fi
  read -n 1 -r
  if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    return 0
  fi

  return 1
}

#
# Input options
#
# Transform long options to short ones
for arg in "$@"; do
  shift
  case "$arg" in
    '--help')               set -- "$@" '-h'   ;;
    '--config')             set -- "$@" '-c'   ;;
    '--no-upload')          set -- "$@" '-n'   ;;
    '--metadata-only')      set -- "$@" '-m'   ;;
    '--upload-only')        set -- "$@" '-u'   ;;
    '--run-as-service')     set -- "$@" '-s'   ;;
    '--kill')               set -- "$@" '-k'   ;;
    '--yes')                set -- "$@" '-y'   ;;
    '--wait-for-jobs-completion') set -- "$@" '-w'   ;;
    *)                      set -- "$@" "$arg" ;;
  esac
done

# Parse short options
# Note that there is no : after w to make the parameter optional but we can only have a single optional parameter
OPTIND=1
while getopts ":hnmusc:kyw" opt
do
  case "$opt" in
    'h') print_usage; exit 0 ;;
    'c') configFilepath="$OPTARG";
          if [[ ! -f "$configFilepath" ]]
          then
            echo "Can't access file $configFilepath !"
            exit 1
          fi ;;
    'n') noUpload="true" ;;
    'm') metadataOnlyMode="true"; heexDataUploaderArgs="$heexDataUploaderArgs --metadata-only" ;;
    'u') uploadOnlyMode="true" ;;
    's') runAsService="true" ;;
    'k') runKill="true" ;;
    'y') promptAnswerIsYes="true" ;;
    'w') waitForJobsCompletion="true";;
    '?') >&2 echo "Unknown argument."; print_usage >&2; exit 1 ;;
  esac
done
shift $(expr $OPTIND - 1) # remove options from positional parameters

# Check for uncohrent options
if [[ "$waitForJobsCompletion" == "true" && "$runKill" == "false" ]]
then
  echo "Please use --kill option with --wait-for-jobs-completion"
  exit 0
fi

#
# Tool functions
#

function waitForJobsCompletionAndKill
{
  numJobs=-1

  if ! command -v "./3rdparty/Sqlite3/sqlite" &> /dev/null
  then
    echo "Error: sqlite need to be compiled for --wait-for-jobs-completion"
    killAllKernelInstances
    exit 0
  fi


  if [[ -f "kernel/$heexDatabaseFile" ]]
  then
    numJobs=$(./3rdparty/Sqlite3/sqlite "kernel/$heexDatabaseFile" "SELECT COUNT(*) FROM jobs;")
  fi

  # check no jobs files
  if [[ $numJobs == -1 ]]
  then
    killAllKernelInstances
    exit 0
  fi

  # Kill the core to prevent generating new events
  killall heexCore >/dev/null 2>&1

  while [[ $numJobs > 0 ]]  # Wait until their is no more jobs to process
  do
    sleep 1
    numJobs=$(./3rdparty/Sqlite3/sqlite "kernel/$heexDatabaseFile" "SELECT COUNT(*) FROM jobs;")
  done

  # Kill data collector and data uploader when jobs finished
  killall heexDataUploader heexDataCollector >/dev/null 2>&1
}
# Kill all processus run using this script (heexCore heexDataCollector heexDataUploader)
function killAllKernelInstances
{
  # Kill all related processes
  if [[ -f "$heexDefaultWorkingDirectory/runHeexKernel.pids" ]]
  then
    kill $(cat "$heexDefaultWorkingDirectory/runHeexKernel.pids")
  else
    # NOTE:
    # ">/dev/null 2>&1" prevents any kill cmd output to be mixed in log as some processes may not exist.
    killall -s SIGTERM heexCore heexDataCollector heexDataUploader >/dev/null 2>&1
    echo "SIGTERM signal have been sent to all Heex Kernel processes (heexCore heexDataCollector heexDataUploader)."
  fi

  # Give some time to the processes to be killed (heexCore can take at least 1s to finished in destructor due to a sleeping thread)
  sleep 2
}

#
# Lauch Kernel or other optional operations
#

# Check if kill Kernel only
if [[ "$runKill" == "true" ]]
then
  if [[ "$waitForJobsCompletion" == "true" ]]
  then
    waitForJobsCompletionAndKill
  else
    killAllKernelInstances
  fi
  exit 0
fi

#
# Checks and operation before execution
#

# Check if the script is run from the Heex SDK folder.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # Works if the path used to find the script is not a symlink https://stackoverflow.com/a/246128
if [[ "$(pwd)" != "$SCRIPT_DIR" ]]
then
  previousDir="$(pwd)"
  cd $SCRIPT_DIR
fi

# Resolve the absolute path to the configuration file with option "--config" not used. Requires "cd $SCRIPT_DIR" cmd to be perform before.
if [[ -z "$configFilepath" ]]
then
  # Make absolute the default or provided filepath to be usable by the Heex Kernel.
  configFilepath=$defaultConfigFilepath

  # Handle backward compatiblity for older sdk versions which may contains systemconf in kernel folder
  if [[ ! -f "$configFilepath" ]]
  then
    if [[ -f "$oldDefaultConfigFilepath" ]]
    then
      # Extract the directory path from the file path
      dirPath=$(dirname "$configFilepath")

      # Create the directory path
      mkdir -p "$dirPath"
      oldDefaultConfigFilepath="$(realpath ${oldDefaultConfigFilepath})"
      mv "$oldDefaultConfigFilepath" "$configFilepath"
    fi
  fi
fi

# Resolve the absolute path to the configuration file
configFilepath="$(realpath ${configFilepath})"

# Check configuration file existence
if [[ ! -f "$configFilepath" ]]
then
  >&2 echo "Can't access config file $configFilepath!"
  exit 1
fi

if [[ ! -z "$previousDir" ]]
then
  cd "$previousDir"
fi

if pgrep -f "heexCore|heexDataCollector|heexDataUploader" &> /dev/null
then
  promptUserForChoice "Kernel is already running. Do you want to kill before run or cancel? No is equivalent to cancel"
  if [[ $? -ne 0 ]]; then
    exit 0
  fi
  killall heexCore heexDataCollector heexDataUploader &> /dev/null

  # kill running builtin agents
  if [[ -d "agents/builtin" ]]
  then
    for FILE in agents/builtin/*; do
      if [[ -x $FILE ]]
      then
        killall $FILE
      elif [[ "${FILE: -3}" == ".py" ]]
      then
        pkill -f $FILE
      fi
    done
  fi
fi

cd kernel

# Run SDE (in kernel)
if [[ "$uploadOnlyMode" == "false" ]]
then
  ./heexCore -c "$configFilepath" &
  sleep 1
  ./heexDataCollector -c "$configFilepath" &
fi

if [[ "$noUpload" == "false" ]]
then
  sleep 1
  ./heexDataUploader -c "$configFilepath" $heexDataUploaderArgs &
fi

cd ..

# loop through files in 'agents/builtin' folder (if exists) and execute them
if [[ "$runAsService" == "false" ]]
then
  if [[ -d "agents/builtin" ]]
  then
    for FILE in agents/builtin/*; do
      if [[ -x $FILE ]]
      then
        ./$FILE &
      elif [[ "${FILE: -3}" == ".py" ]]
      then
        python3 $FILE &
      fi
    done
  fi
fi

# Write all pids in heex user working workspace
mkdir -p "$heexDefaultWorkingDirectory"
jobs -p | tee "$heexDefaultWorkingDirectory/runHeexKernel.pids" >/dev/null

# Go back to initial folder
if [[ "cdPerformed" == "true" ]]
then
  cd $previousDir
fi

if [[ "$runAsService" == "true" ]]
then
# this loop is mandatory to run this script as a service
# this script will be the parent process of the SDE
# if we exit this script the service will be killed instantly
  while true; do
   :
  done
fi
