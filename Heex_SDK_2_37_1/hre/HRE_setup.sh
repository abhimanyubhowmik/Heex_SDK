#!/bin/bash
#
# Copyright (c) 2023 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#

# Manage install and update of all of the Heex executables of the Heex Kernel and additional agents as services.

# Add all logs to install.log to keep trace.
exec > >(tee ./install.log) 2>&1

# Add context logs :
echo "$(uname -a)"
echo "Installation started at $(date)"

# In the following lines, we want to retrieve the path of the self-extracting archive. To do so, we need to go up in the process tree
# After that we can get the full path of the grand parent process (which is the self-extracting archive)
installer_path=$(ps -o args= -q $PPID | awk '{print $2}')
if [[ ! -f "$installer_path" ]]; then
  # in case the installer was called without it's full path
  grand_parent_pid=$(ps -o ppid= -q $PPID)
  grand_parent_working_dir=$(pwdx $grand_parent_pid | awk -F '^[^ ]* ' '{print $2}')

  installer_path=$grand_parent_working_dir/$(echo $installer_path | sed 's:^\./::')
fi
echo "Installer path during installation: $installer_path"

# Global variables
sdeMode=""
installerOsName=""
previousDir=
clientScriptRetVal=
executionMode=""
installerOsVersion=""
installerArchitecture=""
systemConf=""
interactiveInstall="false"
hreDefaultInstallDirectory="/opt/heex"
hreFolder="hre"
hreFolderPath=""
hreConfigFile=""
uninstall="false"
installViaOta="false"
keepHreIsInstallingFile="false"
pythonPath=""
unknownKernelVersion="unknownKernelVersion"
canAccessSystemCtl="true"
kernelInstalledList=()
# Get the username of the user who invoked sudo
currentUser=${SUDO_USER:-$USER}

# agents array parsing constants
uuidOffset=0
versionOffset=1
fileOffset=2
md5Offset=3
frameworkOffset=4
agentServiceNameOffset=5
agentOffset=6

# ANSI color codes for warnings
ORANGE='\033[0;33m'   # Orange color
RED='\033[0;31m'      # Red color
GREEN='\033[0;32m'    # Green color
NOCOLOR='\033[0m'     # No color (reset)

#
# Set The global variable canAccessSystemCtl
#
function checkSystemCtlAccess()
{
  if ! command systemctl &> /dev/null
  then
    canAccessSystemCtl="false"
  fi
}

#
# Generate run process command
#
function runProcess()
{
  local agentPath=$1
  local frameworkName="$2"
  local args=$3

  local runProcessCmd=""

  local agentName="$(basename "$agentPath")"

  local preSourcingCmd=""
  local runProcessCmd=""
  # first we check if user has requested for files to be sourced in the HRE_setup_header file
  if [[ -v FILES_TO_SOURCE_IN_SERVICES_ENV && ! "$agentName" =~ ^(heexCore|heexDataCollector|heexDataUploader)$ ]]; then
    for fileToSource in $FILES_TO_SOURCE_IN_SERVICES_ENV; do
      if [[ -e "$fileToSource" ]]; then
        preSourcingCmd="$preSourcingCmd source '$fileToSource' && "
      else
        printf "${ORANGE}[WARNING] file $fileToSource added in the FILES_TO_SOURCE_IN_SERVICES_ENV in your HRE_setup_header.sh does not exist. It won't be sourced.${NOCOLOR}\n"
      fi
    done
  fi


  if [[ "$frameworkName" == "ros1" || "$frameworkName" == "ros2" ]]; then
    local rosExecCmd=""
    # then we need to find the setup.bash of ros. We will look for all available distributions in /opt/ros and look for the setup.bash
    # If more than one distribution available, we will source them all but add a warning to user
    if [[ -e /opt/ros ]]; then
      local rosDistros=$(ls -1 /opt/ros)
      local numDistros=0

      # now we go and look for specific ros distributions inside /opt/ros and we source it's setup.bash to set up ros env in the service
      for distro in $rosDistros; do
        if [[ -e /opt/ros/$distro/setup.bash ]]; then
          numDistros+=1
          rosExecCmd="$rosExecCmd source /opt/ros/$distro/setup.bash &&"
        fi
      done
      if [[ $numDistros -eq 0 ]]; then
        printf "${ORANGE}[WARNING] no ROS distribution found in /opt/ros folder. Unless you have given specific path to it in the HRE_setup_header, you'll need to install a ROS distribution to run a ROS agent${NOCOLOR}\n"
      elif [[ $numDistros -gt 1 ]]; then
        printf "${ORANGE}[WARNING] more than one ROS distribution has been found in /opt/ros folder. We will be sourcing all setup.bash files which could result in some environment variable issues${NOCOLOR}\n"
      fi
    else
      printf "${ORANGE}[WARNING] /opt/ros folder has not been found. Unless you have given specific path to it in the HRE_setup_header, you'll need to install a ROS distribution to run a ROS agent${NOCOLOR}\n"
    fi
    # And finally add the execution of the agent in the command
    rosExecCmd="$rosExecCmd'$agentPath'"
    runProcessCmd="$preSourcingCmd $rosExecCmd"

  elif [[ "${agentPath: -3}" == ".py" ]]; then
    if [[ -z $pythonPath ]]; then
      echo "Python3 is not installed on your machine, ${agentPath} won't be able to be launch as a background process."
      return 1
    fi
    if [[ -n "$args" ]]; then
      runProcessCmd="$preSourcingCmd '$pythonPath' '$agentPath' '$args'"
    else
      # Purposely avoid using args if empty to prevent it to be taken as an empty argument.
      runProcessCmd="$preSourcingCmd '$pythonPath' '$agentPath'"
    fi
  else
    if [[ -n "$args" ]]; then
      runProcessCmd="$preSourcingCmd '$agentPath' '$args'"
    else
      # Purposely avoid using args if empty to prevent it to be taken as an empty argument.
      runProcessCmd="$preSourcingCmd '$agentPath'"
    fi
  fi
  # Execute the command as the non-root user EXCEPT for heexDataUploader since it needs to be able to update HRE
  if [[ "$agentName" == "heexDataUploader" ]]; then
    eval "$runProcessCmd &> /dev/null &"  # running in sudo 
  else
    commandToRun="eval \"$runProcessCmd &> /dev/null &\""
    sudo -u "$currentUser" bash -c "$commandToRun"  # running non root
  fi
  if [[ $? -ne 0 ]]; then
    printf "${RED}[ERROR] ${agentName} failed to launch. Check install.log for more information${NOCOLOR}\n"
    return 1
  fi
  
  maxWaitTime=2  # Maximum wait time in seconds
  currentWaitTime=0
  while [[ $currentWaitTime -lt $((maxWaitTime * 10)) ]]; do
    PID="$(pgrep -f "$agentPath")"  # try to retrieve PID
    if [[ ! -z "$PID" ]]; then # pid found
      break
    fi
    ((currentWaitTime += 1))
    sleep 0.1
  done
  echo "$PID" > "$agentPath.pid"
  return 0
}

#
# Help & Usage
#
function print_usage()
{
  echo "The installer was built for ${installerOsName} ${installerOsVersion} with target architecture ${installerArchitecture}"
  echo "Usage: ./<installer_file> -- [option]"
  echo "-h [ --help]                          To get usage and additional info"
  echo "-m [ --mode]                          Set the HRE mode. Possibilities are upload-only(u) | no-connectivity(n) | limited-connectivity(l)"
  echo "-i [ --interactive]                   Enable interactive mode during installation."
  echo "-c [ --system-conf]                   Set the system configuration file to use, overwritting the default one. NEED AN ABSOLUTE PATH"
  echo "-u [ --uninstall]                     Uninstall"
  echo "-e [ --execution]                     Set the execution mode to either 'background' for background process or 'service'. Default will be service if environment allows it, else background process"
  # echo "-z [ --ota-update]                    Runs the HRE OTA update process"   # not shown in helper function for now, should only be called via scripts, not manually.
}

#
# Handling error or success behaviour of script
#
function exitBehaviour()
{
  local success=$1
  if [[ "$interactiveInstall" == "true" ]]; then
    read -n 1 -s -r -p "Press any key to continue"$'\n'
  fi

  if ! [[ $(id -u) = 0 ]]; then
    # we don't have root privileges, so we cannot copy the install log to hreFolderPath
    tmpHeexlogFolder="/tmp/heexlog"
    if [[ ! -d "$tmpHeexlogFolder" ]]; then
      mkdir -p "$tmpHeexlogFolder"
    fi
    cp install.log "$tmpHeexlogFolder/"
    echo "The install.log can be found here : $tmpHeexlogFolder/install.log"
  else
    # if we have root privileges, we set the log in the hreFolder (and create the folder if it doesn't exist)
    if [[ ! -d "$hreFolderPath" ]]; then
      mkdir -p "$hreFolderPath"
    fi
    cp install.log "$hreFolderPath/"
    if [[ "$success" == "false" ]]; then
      echo "The install.log can be found here : $hreFolderPath/install.log"
    fi
  fi

  # delete the hreIsInstalling file in the installer folder if this was a manual install (ie not OTA)
  if [[ "$keepHreIsInstallingFile" == "false" && "$installViaOta" == "false" && -f "$hreFolderPath/kernel/hreInstallers/hreIsInstalling" ]]; then
    rm "$hreFolderPath/kernel/hreInstallers/hreIsInstalling"
  fi

  if [[ "$success" == "true" ]]; then
    exit 0;
  else
    exit 1;
  fi
}

function promptUserForChoice ()
{
  local message=$1
  echo "$message [Y/n]"

  if [[ "$interactiveInstall" == "false" ]]; then
    echo "Autoreply Yes"
    return 0
  fi
  read -n 1 -r
  if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    return 0
  fi

  return 1
}

function installEssentials ()
{
  # lsb_release
  # NOTE The following block is the same as the one in boostrap.sh, therefore needed to be updated if updated in boostrap.sh
  if [[ ! -z $(cat /etc/os-release | { grep "rhel" || true; }) ]]
  then
    # Red Hat Enterprise Linux (rhel) OS
    if [[ -z $(rpm -qa redhat-lsb-core | grep "redhat-lsb-core") ]]
    then
      # Install package needed for the lsb_release command
      yum install redhat-lsb-core -y
    fi
  else
    # Debian
    if ! command -v lsb_release &> /dev/null
    then
      echo "The 'lsb-core' package isn't installed. Installing 'lsb-core' using 'apt install lsb-core -y'"
      apt install lsb-core -y
    fi
  fi
}

# Check that the installer was built on a computer that matches the current machine where it is run
function checkInstallerRequirements ()
{
  if [[ $(command -v lsb_release) ]]; then
    local currentOsName=$(echo $(lsb_release -d) | awk '{ print $2 }')
    if [[ "${currentOsName}" != "${installerOsName}" ]]; then
      toDisplay="Your system (${currentOsName}) doesn’t match the one of this installer (${installerOsName}). Continue anyway (y/n)?";
      promptUserForChoice "$toDisplay"
    fi
    local currentOsVersion=$(echo $(lsb_release -r) | awk '{ print $2 }')
    if [[ "$currentOsName" == "CentOS" ]]; then
      # Only keep the Major and Minor of the CentOs version numbers : xx.yy.zz -> xx.yy
      currentOsVersion=$(echo $(lsb_release -r) | awk '{ print $2 }' | cut -d. -f1,2)
    fi
    if [[ "${currentOsVersion}" != "${installerOsVersion}" ]]; then
      toDisplay="Your system version (${currentOsVersion}) doesn’t match the one of this installer (${installerOsVersion}). Continue anyway (y/n)?"
      promptUserForChoice "$toDisplay"
    fi
  else
    echo "OS name and OS version check not done due to missing lsb_release package. We assume that you know what you are doing so the installation will continue"
  fi

  local currentOsArchi=""
  local currentOsArchiTmp="$(uname -m)"
  if [[ "${currentOsArchiTmp}" == "x86_64" ]]; then
    currentOsArchi=${currentOsArchiTmp}
  elif [[ "${currentOsArchiTmp}" == "i686" ]] | [[ "${currentOsArchiTmp}" == "i386" ]] | [[ "${currentOsArchiTmp}" == "x86" ]]; then
    currentOsArchi="x86"
  elif [[ "${currentOsArchiTmp}" == "aarch32" ]] | [[ "${currentOsArchiTmp}" == "aarch" ]]; then
    currentOsArchi="aarch"
  elif [[ "${currentOsArchiTmp}" == "aarch64" ]]; then
    currentOsArchi="aarch64"
  else
    echo "The current OS architecture is not listed in the supported UNIX OS."
  fi

  if [[ "$currentOsArchi" != "$installerArchitecture" ]]; then
    toDisplay="Your system architecture (${currentOSArchi}) doesn’t match the one of this installer (${installerArchitecture}). Continue anyway?"
    promptUserForChoice "$toDisplay"
    if [[ $? -ne 0 ]]; then
      exitBehaviour "false"
    fi
  fi

  if ! [[ $(id -u) = 0 ]]; then
    echo "The Installer needs to be executed with root privileges."
    exitBehaviour "false";
  fi

  # Python checks
  if ! [[ -z $(python3 -V) ]]; then
    pythonPath="$(which python3)"
  fi
}

function getExecServiceName()
{
  local execServiceName=""
  local execName="$(basename "$1")"
  if [[ "$execName" == "heexCore" || "$execName" == "heexDataCollector" || "$execName" == "heexDataUploader" ]];
  then
    execServiceName="$execName"
  else
    execServiceName="agent${execName^}"
    execServiceName="${execServiceName// /_}"
  fi
  echo "$execServiceName"
}

function checkServiceStatus ()
{
  local serviceName="$1"
  local status=""
  local serviceFile=""

  serviceFile="/etc/systemd/system/$serviceName.service"
  if [[ ! -f "$serviceFile" ]]; then
    status="ToDeploy"
    # handle edge case where service might still be running
    systemctl stop $serviceName
    systemctl disable $serviceName
  else
    if ! systemctl is-active --quiet "$serviceName"; then
      status="ToRestart"
    else
      status="running"
    fi
  fi
  echo "'$status'"
}

function checkBackgroundProcessStatus ()
{
  local installDir=$1
  local agentRelativePath=$2

  local execAbsolutePath="$installDir/agents/${agentRelativePath: +2}"
  # Kernel executables are treated differently than agents. Kernel processes do not follow the agent naming convention. Therefore, the if condtion
  if [[ "$agentRelativePath" == "heexCore" || "$agentRelativePath" == "heexDataCollector" || "$agentRelativePath" == "heexDataUploader" ]]; then
    execAbsolutePath="$installDir/kernel/$agentRelativePath"
  fi

  if ! ps -ax  | grep -v grep | grep -q "$execAbsolutePath"; then
    status="ToRestart"
  else
    status="running"
  fi

  echo "'$status'"
}

function checkExecutionStatus ()
{
  local serviceName=$1
  local __result=$2
  local installDir=$3
  local agentRelativePath=$4

  if [[ "$executionMode" == "service" ]]; then
    status=$(checkServiceStatus "$serviceName")
  else
    status=$(checkBackgroundProcessStatus "$installDir" "$agentRelativePath")
  fi
  eval $__result="'$status'"
}

function addService()
{
  local agentPath="$1"
  local args="$2"
  local currentWorkingDirectory="$3"
  local execServiceName="$4"
  local frameworkName="$5"

  local serviceFileName="$execServiceName.service"
  local serviceFilePath="/etc/systemd/system/$serviceFileName"

  local ExecStartCmd=""
  local preSourcingCmd=""
  # For services which ExecStart is multi command, it is done through a terminal using cmd below
  local multiCmdExecStartCmd="/bin/bash -c"

  # first we check if user has requested for files to be sourced in the HRE_setup_header file
  if [[ -v FILES_TO_SOURCE_IN_SERVICES_ENV && ! "$execServiceName" =~ ^(heexCore|heexDataCollector|heexDataUploader)$ ]]; then
    for fileToSource in $FILES_TO_SOURCE_IN_SERVICES_ENV; do
      if [[ -e "$fileToSource" ]]; then
        preSourcingCmd="$preSourcingCmd source \\\"$fileToSource\\\" && "  # adding \\\"\\\" to support any spaces in paths
      else
        printf "${ORANGE}[WARNING] file $fileToSource added in the FILES_TO_SOURCE_IN_SERVICES_ENV in your HRE_setup_header.sh does not exist. It won't be sourced.${NOCOLOR}\n"
      fi
    done
  fi

  if [[ "$frameworkName" == "ros1" || "$frameworkName" == "ros2" ]]; then
    local rosExecCmd=""
    # then we need to find the setup.bash of ros. We will look for all available distributions in /opt/ros and look for the setup.bash
    # If more than one distribution available, we will source them all but add a warning to user
    if [[ -e /opt/ros ]]; then
      local rosDistros=$(ls -1 /opt/ros)
      local numDistros=0

      # now we go and look for specific ros distributions inside /opt/ros and we source it's setup.bash to set up ros env in the service
      for distro in $rosDistros; do
        if [[ -e /opt/ros/$distro/setup.bash ]]; then
          numDistros+=1
          rosExecCmd="$rosExecCmd source /opt/ros/$distro/setup.bash &&"
        fi
      done
      if [[ $numDistros -eq 0 ]]; then
        printf "${ORANGE}[WARNING] no ROS distribution found in /opt/ros folder. Unless you have given specific path to it in the HRE_setup_header, you'll need to install a ROS distribution to run a ROS agent${NOCOLOR}\n"
      elif [[ $numDistros -gt 1 ]]; then
        printf "${ORANGE}[WARNING] more than one ROS distribution has been found in /opt/ros folder. We will be sourcing all setup.bash files which could result in some environment variable issues${NOCOLOR}\n"
      fi
    else
      printf "${ORANGE}[WARNING] /opt/ros folder has not been found. Unless you have given specific path to it in the HRE_setup_header, you'll need to install a ROS distribution to run a ROS agent${NOCOLOR}\n"
    fi
    # And finally add the execution of the agent in the command
    rosExecCmd="$rosExecCmd \\\"$agentPath\\\""
    ExecStartCmd="$multiCmdExecStartCmd '$preSourcingCmd$rosExecCmd'"
  else
    # Special treatment for Python scripts
    if [[ "${agentPath: -3}" == ".py" ]]; then
      if [[ -z $pythonPath ]]; then
        echo "Python3 is not installed on your machine, ${agentPath} won't be able to be launch as service."
        return 1
      fi
      pythonAgentExecStartCmd="\\\"$pythonPath\\\" \\\"$agentPath\\\""
      ExecStartCmd="$multiCmdExecStartCmd '$preSourcingCmd$pythonAgentExecStartCmd'"

    else
      ExecStartCmd="$multiCmdExecStartCmd '$preSourcingCmd$agentPath'"
    fi
  fi
  if [[ ! -z "$args" ]]; then
    ExecStartCmd="$ExecStartCmd '$args'"
  fi

  # Execute the command as the non-root user EXCEPT for heexDataUploader since it needs to be able to update HRE
  serviceUser=$currentUser
  if [[ "$execServiceName" == "heexDataUploader" ]]; then
    serviceUser=$USER # sudo user
  fi

  echo -e "\nAdding $execServiceName as a startup service ..."

  systemctl daemon-reload

  bash -c "echo \"[Unit]
  Description=$execServiceName daemon

  [Service]
  Type=simple
  User=$serviceUser
  WorkingDirectory=$currentWorkingDirectory
  ExecStart=$ExecStartCmd
  Restart=on-failure

  [Install]
  WantedBy=default.target\" > $serviceFilePath"

  systemctl enable $serviceFileName
  if [[ $? -ne 0 ]]; then
    return 1
  fi
  systemctl daemon-reload
  if [[ $? -ne 0 ]]; then
    return 1
  fi
  systemctl reset-failed
  if [[ $? -ne 0 ]]; then
    return 1
  fi
  echo "$execServiceName has been successfully added as a service as '$serviceFileName'."
  echo "Start service $serviceFileName"
  systemctl start $serviceFileName
  return $?
}

function addBackgroundProcess ()
{
  local agentPath=$1
  local args=$2
  local currentWorkingDirectory=$3
  local frameworkName="$4"

  local agentName=$(basename "$agentPath")
  echo -e "\nAdding "$agentName" as a background process ..."

  cd "$currentWorkingDirectory"
  runProcess "$agentPath" "$frameworkName" "$args"
  if [[ $? -ne 0 ]]; then
    cd - &> /dev/null
    return 1
  fi

  cd - &> /dev/null
  return 0
}

function addExecutable ()
{
  local execRelativePath="$1"  # Path relative  to the executable location
  local serviceName="$2"
  local frameworkName="$3"     # framework (ie: informs us if we are in are running ROS service)
  local installDir="$4"
  local args="$5"              # Arguments for the agent

  local isPythonAgent="false"
  local agentPath=""
  local currentWorkingDirectory=""
  # Special treatment for Kernel executables
  if [[ "$execRelativePath" == "heexCore" || "$execRelativePath" == "heexDataCollector" || "$execRelativePath" == "heexDataUploader" ]]; then
    agentPath="$installDir/kernel/$execRelativePath"
    currentWorkingDirectory="$installDir/kernel"
  else
    agentPath="$installDir/agents/${execRelativePath: +2}"  # (ignore first two './')
    currentWorkingDirectory="$(dirname "$agentPath")"
    cp "$PWD/agents/${execRelativePath: +2}" "$installDir/agents/${execRelativePath: +2}"
    # Ownership is required before running the service, as the service are run with current user, however $installDir is owned by root
    chown -R "$currentUser":"$currentUser" "$installDir"
  fi

  if [[ "$executionMode" == "service" ]]; then
    addService "$agentPath" "$args" "$currentWorkingDirectory" "$serviceName" "$frameworkName"
  else

    addBackgroundProcess "$agentPath" "$args" "$currentWorkingDirectory" "$frameworkName"
  fi

  return $?
}

function removeService()
{
  local serviceName=$1
  # Delete service associated to the service
  if [[ -f "/etc/systemd/system/$serviceName.service" ]]; then
    systemctl stop $serviceName
    systemctl disable $serviceName
    rm -f "/etc/systemd/system/$serviceName.service"

    systemctl daemon-reload
    systemctl reset-failed
  fi
  return 0
}

function removeBackgroundProcess()
{
  local executableAbsolutePath="$1"
  if [[ -f "$executableAbsolutePath.pid" ]]; then
    kill $(cat "$executableAbsolutePath.pid") &> /dev/null
    rm -f "$executableAbsolutePath.pid"
  fi
  return 0
}

function removeExecutable()
{
  local executableRelativePath="$1"
  local serviceName="$2"
  local installDir="$3"

  local executableAbsolutePath=""

  if [[ "$executableRelativePath" == "heexCore" || "$executableRelativePath" == "heexDataCollector" || "$executableRelativePath" == "heexDataUploader" ]]; then
    if [[ -f "$installDir/kernel/$executableRelativePath" ]]; then
      executableAbsolutePath="$installDir/kernel/$executableRelativePath"
    fi
  else
    # (ignore first two './')
    if [[ -f "$installDir/agents/${executableRelativePath: +2}" ]]; then
      executableAbsolutePath="$installDir/agents/${executableRelativePath: +2}"
    fi
  fi
  if [[ "$executionMode" == "service" ]]; then
    removeService "$serviceName"
  else
    removeBackgroundProcess "$executableAbsolutePath"
  fi
  # Delete executable
  if [[ -f "$executableAbsolutePath" ]]; then
    rm -f "$executableAbsolutePath"
  fi
  return 0
}

function replaceService()
{
  serviceName="$1"
  if [[ -f "/etc/systemd/system/$serviceName.service" ]]
  then
    systemctl restart $serviceName
  fi
  return 0
}

function replaceBackgroundProcess()
{
  local executableAbsolutePath="$1"
  local frameworkName="$2"
  local args="$3"

  local isPythonAgent=""
  if [[ -f "$executableAbsolutePath.pid" ]]; then
    local currentWorkingDirectory="$(dirname "$executableAbsolutePath")"
    cd "$currentWorkingDirectory"
    kill $(cat "$executableAbsolutePath.pid") &> /dev/null
    runProcess "$executableAbsolutePath" "$frameworkName" "$args"
    echo "$!" > "$executableAbsolutePath.pid"
    cd - &> /dev/null
  fi
  return 0
}

function replaceExecutable()
{
  local executableRelativePath="$1"
  local serviceName="$2"
  local extractionDir="$3"
  local frameworkName="$4"
  local installDir="$5"
  local args="$6"

  local executableAbsolutePath=""
  local currentWorkingDirectory=""

  # Replace executable
  if [[ "$executableRelativePath" == "heexCore" || "$executableRelativePath" == "heexDataCollector" || "$executableRelativePath" == "heexDataUploader" ]]; then
    executableAbsolutePath="$installDir/kernel/$executableRelativePath"
    rm -f "$executableAbsolutePath"
    cp "$extractionDir/kernel/$executableRelativePath" "$executableAbsolutePath"
  else
    # (ignore first two './')
    executableAbsolutePath="$installDir/agents/${executableRelativePath: +2}"
    rm -f "$executableAbsolutePath"
    cp "$extractionDir/agents/${executableRelativePath: +2}" "$executableAbsolutePath"
  fi
  # Ownership is required before running the service, as the service are run with current user, however $hreFolderPath is owned by root
  chown -R "$currentUser":"$currentUser" "$executableAbsolutePath"

  if [[ "$executionMode" == "service" ]]; then
    replaceService "$serviceName"
  else
    replaceBackgroundProcess "$executableAbsolutePath" "$frameworkName" "$args"
  fi

  return $?
}

function restartService()
{
  serviceName="$1"
  systemctl start $serviceName
  return $?
}

function restartBackgroundProcess()
{
  local installDir="$1"
  local executableRelativePath="$2"
  local frameworkName="$3"
  local args="$4"

  local isPythonAgent=""

  # Only agents services restart are handled
  local executableAbsolutePath="$installDir/agents/${executableRelativePath: +2}"
  if [[ -f "$executableAbsolutePath.pid" ]]; then
    kill $(cat "$executableAbsolutePath.pid") &> /dev/null
    rm -f "$executableAbsolutePath.pid"
  fi
  local currentWorkingDirectory="$(dirname "$executableAbsolutePath")"
  cd "$currentWorkingDirectory"
  runProcess "$executableAbsolutePath" "$frameworkName" "$args"
  echo "$!" > "$executableAbsolutePath.pid"
  cd - &> /dev/null
  return 0
}

function restartExecutable()
{
  local executableRelativePath="$1"   # Path relative from the inside of "agents" folder
  local serviceName="$2"
  local frameworkName="$3"
  local installDir="$4"
  local args="$5"                     # Arguments for the agent

  if [[ "$executionMode" == "service" ]]; then
    restartService "$serviceName"
  else
    restartBackgroundProcess "$installDir" "$executableRelativePath" "$frameworkName" "$args"
  fi
  return $?
}

function parseAgentFile()
{
  local agentJsonFile="$1"
  local agentResultArr=()
  local agentsBlock

  # Read the agents block from the JSON file
  agentsBlock=$(awk '/"agents": \[/{flag=1;next}/\]/{flag=0}flag' "$agentJsonFile")
  local json_objects=$(echo $agentsBlock | sed -n 's/}/}\n/gp')

  # Process each JSON object
  while IFS= read -r json_object; do
    # Skip processing if the line does not contain "uuid"
    if [[ ! $json_object =~ "uuid" ]]; then
      continue
    fi
    # Extracting each field using sed
    local uuid=$(echo $json_object | sed -n 's/.*"uuid": "\([^"]*\)".*/\1/p')
    local version=$(echo $json_object | sed -n 's/.*"version": "\([^"]*\)".*/\1/p')
    local file=$(echo $json_object | sed -n 's/.*"file": "\([^"]*\)".*/\1/p')
    local md5=$(echo $json_object | sed -n 's/.*"md5": "\([^"]*\)".*/\1/p')
    local framework=$(echo $json_object | sed -n 's/.*"framework": "\([^"]*\)".*/\1/p')

    local agentServiceName=$(getExecServiceName "$file")

    # Add the extracted fields to the array
    agentResultArr+=("$uuid" "$version" "$file" "$md5" "$framework" "$agentServiceName")
  done <<< "$json_objects"

  # we separate all the elements with '\n' instead of spaces to support spaces in the names of the elements.
  printf '%s\n' "${agentResultArr[@]}"
}

function getTargetIndexInArray()
{
  target="$1"
  searchArr=($2)

  for (( i=0; i<${#searchArr[@]}; i++ )); do
    if [[ "${searchArr[$i]}" == "$target" ]]; then
      echo "$i"
      return 0
    fi
  done
  echo "-1"
  return 0
}

function removeAgents()
{
  local installDir="$1"
  local agentOldList=()
  local agentsToRemove=()

  if [[ -f "$installDir/agents/agents.json" ]]; then
    # since parseAgentFile returns each element seperated with \n, we have to fetch the elements accordingly using Internal Field Separator
    IFS=$'\n' read -r -d '' -a agentOldList < <(parseAgentFile "$installDir/agents/agents.json")
  fi

  for (( oldIndex=0; oldIndex<${#agentOldList[*]}; oldIndex=oldIndex+$agentOffset )); do
    for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
      agentsToRemove+=("${agentOldList[$oldIndex+$offsetIndex]}")
    done
  done

  for (( i=0; i<${#agentsToRemove[*]}; i=i+$agentOffset )); do
    oldIndex=$(getTargetIndexInArray "${agentsToRemove[$i]}" "${agentOldList[*]}")
    removeExecutable "${agentsToRemove[$i+$fileOffset]}" "${agentsToRemove[$i+$agentServiceNameOffset]}" "$installDir"
    echo "${agentsToRemove[$i+$agentServiceNameOffset]} ${agentsToRemove[$i]}(${agentOldList[$oldIndex+$versionOffset]}) removed."
  done
}

function installAgents()
{
  local extractionDir="$1"  # Contains new agents to install
  local installDir="$2"     # Contains possibly older agents
  local installType="$3"    # Caller determine if a fresh install must be performed or not. True if "freshInstall" is passed
  echo -e "\nInstalling agents for a $installType"

  # Load json values
  local agentOldList=()
  local agentNewList=()
  # since parseAgentFile returns each element seperated with \n, we have to fetch the elements accordingly using Internal Field Separator
  IFS=$'\n' read -r -d '' -a agentNewList < <(parseAgentFile "$extractionDir/agents/agents.json")

  if [[ "$installType" == "fresh install" ]]; then
    # Delete current installed agents, and install new agents present in agent folder of extractionDir
    removeAgents "$installDir"
    mkdir -p "$installDir/agents"
    cp -r "$extractionDir/agents" "$installDir"
  else
    # since parseAgentFile returns each element seperated with \n, we have to fetch the elements accordingly using Internal Field Separator
    IFS=$'\n' read -r -d '' -a agentOldList < <(parseAgentFile "$installDir/agents/agents.json")
  fi

  # Identify candidate agents for addition and breaking (removal, replacement) changes and queue jobs
  echo "Computing changes ..."
  local agentsNoChange=()
  local agentsToDeploy=()
  local agentsToRemove=()
  local agentsToReplace=()
  local agentsToRestart=()
  local executablesAlreadyDeployed=()
  for (( newIndex=0; newIndex<${#agentNewList[*]}; newIndex=newIndex+$agentOffset )); do

    oldIndex=$(getTargetIndexInArray "${agentNewList[$newIndex]}" "${agentOldList[*]}")

    if [[ $oldIndex -ne -1 ]]; then
      if [[ "${agentOldList[$oldIndex+$versionOffset]}" == "${agentNewList[$newIndex+$versionOffset]}" ]]; then
        for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
          agentsNoChange+=("${agentNewList[$newIndex+$offsetIndex]}")
        done
      else
        for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
          agentsToReplace+=("${agentNewList[$newIndex+$offsetIndex]}")
        done
      fi
    else
      for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
        agentsToDeploy+=("${agentNewList[$newIndex+$offsetIndex]}")
      done
    fi
  done
  for (( oldIndex=0; oldIndex<${#agentOldList[*]}; oldIndex=oldIndex+$agentOffset )); do

    newIndex=$(getTargetIndexInArray "${agentOldList[$oldIndex]}" "${agentNewList[*]}")
    if [[ $newIndex -eq -1 ]]; then
      for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
        agentsToRemove+=("${agentOldList[$oldIndex+$offsetIndex]}")
      done
    fi
  done

  # Check already installed agents status
  local agentsNoChangeTmp=("${agentsNoChange[@]}")
  for (( i=0; i<${#agentsNoChangeTmp[*]}; i=i+$agentOffset )); do
    checkExecutionStatus "${agentsNoChangeTmp[$i+$agentServiceNameOffset]}" agentServiceStatus "$installDir" "${agentsNoChangeTmp[$i+$fileOffset]}"
    case "$agentServiceStatus" in
      "ToDeploy")
        for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
          agentsToDeploy+=("${agentsNoChangeTmp[$i+$offsetIndex]}")
          unset agentsNoChange[$i+$offsetIndex]
        done
        ;;
      "ToRestart")
        for (( offsetIndex=0; offsetIndex<$agentOffset; offsetIndex++ )); do
          agentsToRestart+=("${agentsNoChangeTmp[$i+$offsetIndex]}")
          unset agentsNoChange[$i+$offsetIndex]
        done
        ;;
    esac
  done

  # Display a Summary of all changes
  for (( i=0; i<${#agentsNoChange[*]}; i=i+$agentOffset )); do
    echo "${agentsNoChange[$i+$agentServiceNameOffset]} ${agentsNoChange[$i]} (${agentsNoChange[$i+$versionOffset]}) will remain installed."
  done
  for (( i=0; i<${#agentsToReplace[*]}; i=i+$agentOffset )); do
    oldIndex=$(getTargetIndexInArray "${agentsToReplace[$i]}" "${agentOldList[*]}")
    echo "${agentsToReplace[$i+$agentServiceNameOffset]} ${agentsToReplace[$i]} (${agentOldList[$oldIndex+$versionOffset]}) will be replaced by (${agentsToReplace[$i+$versionOffset]})"
  done
  for (( i=0; i<${#agentsToRemove[*]}; i=i+$agentOffset )); do
    echo "${agentsToRemove[$i+$agentServiceNameOffset]} ${agentsToRemove[$i]} (${agentsToRemove[$i+$versionOffset]}) will be removed."
  done
  for (( i=0; i<${#agentsToDeploy[*]}; i=i+$agentOffset )); do
    echo "${agentsToDeploy[$i+$agentServiceNameOffset]} ${agentsToDeploy[$i]} (${agentsToDeploy[$i+$versionOffset]}) will be installed."
  done
  for (( i=0; i<${#agentsToRestart[*]}; i=i+$agentOffset )); do
    echo "${agentsToRestart[$i+$agentServiceNameOffset]} ${agentsToRestart[$i]} (${agentsToRestart[$i+$versionOffset]}) will be restarted."
  done
  # Check or request user acceptance
  promptUserForChoice "The precedent changes will be applied. Do you want to proceed?"
  if [[ $? -ne 0 ]]; then
    echo "HRE update of agents has been cancelled by the user. No agent have been installed nor changed."
    exitBehaviour "false"
  fi
  # fi

  # Applying changes accepted by the user
  echo -e "\nApplying changes ..."
  for (( i=0; i<${#agentsNoChange[*]}; i=i+$agentOffset )); do
    echo "${agentsNoChange[$i+$agentServiceNameOffset]} ${agentsNoChange[$i]}(${agentsNoChange[$i+$versionOffset]}) untouched."
  done

  for (( i=0; i<${#agentsToReplace[*]}; i=i+$agentOffset )); do
    oldIndex=$(getTargetIndexInArray "${agentsToReplace[$i]}" "${agentOldList[*]}")
    executablesAlreadyDeployed+=("${agentsToReplace[$i+$fileOffset]}")
    replaceExecutable "${agentsToReplace[$i+$fileOffset]}" "${agentsToReplace[$i+$agentServiceNameOffset]}" "$extractionDir" "${agentsToDeploy[$i+$frameworkOffset]}" "$installDir"
    if [[ $? -ne 0 ]]; then
      printf "${RED}[ERROR] Failed to replace ${agentsToReplace[$i+$agentServiceNameOffset]} to (${agentsToReplace[$i+$versionOffset]})${NOCOLOR}\n"
    else
      #TODO: Add support for modification of already deployed agent's arguments
      echo "${agentsToReplace[$i+$agentServiceNameOffset]} ${agentsToReplace[$i]}(${agentOldList[$oldIndex+$versionOffset]}) replaced by (${agentsToReplace[$i+$versionOffset]})"
    fi
  done

  for (( i=0; i<${#agentsToRestart[*]}; i=i+$agentOffset )); do
    executablesAlreadyDeployed+=("${agentsToRestart[$i+$fileOffset]}")
    restartExecutable "${agentsToRestart[$i+$fileOffset]}" "${agentsToRestart[$i+$agentServiceNameOffset]}" "${agentsToDeploy[$i+$frameworkOffset]}" "$installDir" "$args"
    if [[ $? -eq 0 ]]; then
      echo "${agentsToRestart[$i+$agentServiceNameOffset]} ${agentsToRestart[$i]}(${agentsToRestart[$i+$versionOffset]}) newly restarted."
    else
      printf "${RED}[ERROR] Failed to restart ${agentsToRestart[$i+$agentServiceNameOffset]}${NOCOLOR}\n"
    fi
  done

  for (( i=0; i<${#agentsToDeploy[*]}; i=i+$agentOffset )); do
    if [[ $(getTargetIndexInArray "${agentsToDeploy[$i+$fileOffset]}" "${executablesAlreadyDeployed[*]}") -eq -1 ]]; then
      executablesAlreadyDeployed+=("${agentsToDeploy[$i+$fileOffset]}")
      addExecutable "${agentsToDeploy[$i+$fileOffset]}" "${agentsToDeploy[$i+$agentServiceNameOffset]}" "${agentsToDeploy[$i+$frameworkOffset]}" "$installDir" "$args"
      if [[ $? -eq 0 ]]; then
        echo "${agentsToDeploy[$i+$agentServiceNameOffset]} ${agentsToDeploy[$i]}(${agentsToDeploy[$i+$versionOffset]}) newly installed."
      else
        printf "${RED}[ERROR] Failed to install ${agentsToDeploy[$i+$agentServiceNameOffset]}${NOCOLOR}\n"

      fi
    else
      echo "${agentsToDeploy[$i+$agentServiceNameOffset]} ${agentsToDeploy[$i]}(${agentsToDeploy[$i+$versionOffset]}) recently installed."
    fi
  done

  for (( i=0; i<${#agentsToRemove[*]}; i=i+$agentOffset )); do
    if [[ $(getTargetIndexInArray "${agentsToRemove[$i+$fileOffset]}" "${executablesAlreadyDeployed[*]}") -eq -1 ]]; then
      oldIndex=$(getTargetIndexInArray "${agentsToRemove[$i]}" "${agentOldList[*]}")
      removeExecutable "${agentsToRemove[$i+$fileOffset]}" "${agentsToRemove[$i+$agentServiceNameOffset]}" "$installDir"
      echo "${agentsToRemove[$i+$agentServiceNameOffset]} ${agentsToRemove[$i]}(${agentOldList[$oldIndex+$versionOffset]}) removed."
    else
      echo "${agentsToRemove[$i+$agentServiceNameOffset]} is used by one or more agents and will remain active."
    fi
  done
  # Overwrite older agents.json with new one
  cp "$extractionDir/agents/agents.json" "$installDir/agents/agents.json"

  echo "Verifying launched agents status:"
  maxWaitTime=10    # seconds. We allow a total of maxWaitTime seconds for all the agents to be running
  currentWaitTime=0 # seconds
  for (( i=0; i<${#agentNewList[*]}; i=i+$agentOffset )); do
    checkExecutionStatus "${agentNewList[$i+$agentServiceNameOffset]}" agentServiceStatus "$installDir" "${agentNewList[$i+$fileOffset]}"
    while [[ $currentWaitTime -lt $((maxWaitTime * 10)) ]]; do
      sleep 0.1
      ((currentWaitTime += 1))
      # as long as agent is not running, we loop
      if [[ "$agentServiceStatus" == "running" ]]; then
        # service is running.
        break
      fi
      checkExecutionStatus "${agentNewList[$i+$agentServiceNameOffset]}" agentServiceStatus "$installDir" "${agentNewList[$i+$fileOffset]}"
    done
    case "$agentServiceStatus" in
      "running")
        printf "${GREEN} ${agentNewList[$i+$agentServiceNameOffset]} ${agentNewList[$i+$uuidOffset]} (${agentNewList[$i+$versionOffset]}) RUNNING ${NOCOLOR}\n"
        ;;
      *)
        printf "${RED} ${agentNewList[$i+$agentServiceNameOffset]} ${agentNewList[$i+$uuidOffset]} (${agentNewList[$i+$versionOffset]}) NOT RUNNING. Check the install.log file and the agent log inside $installDir/agents for more information. ${NOCOLOR}\n"
        ;;
    esac
  done
}

# add Heex Kernel services
function installKernel()
{
  local extractionDir="$1"     # Temporary dir
  local installDir="$2"
  local installType="$3"

  # update kernel. kernel is everything that is not in agents folder (i.e kernel folder, 3rd-party,...)

  promptUserForChoice "The HRE kernel will be updated (if not present, the installation will be done). Do you want to proceed?"
  if [[ $? -ne 0 ]]; then
    echo "HRE kernel update has been cancelled by the user. No modifications have been made"
    exitBehaviour "false"
  fi

  # Delete Kernel services
  removeExecutable "heexCore" "heexCore" "$installDir"
  removeExecutable "heexDataCollector" "heexDataCollector" "$installDir"
  removeExecutable "heexDataUploader" "heexDataUploader" "$installDir"

  # agents will by handled by installAgents
  for folder in "$extractionDir"/*; do
    if [[ -d "$folder" && "$(basename "$folder")" == "agents" ]]; then
      continue
    fi
    cp -r "$folder" "$installDir"
    if [[ -d "$folder" && "$(basename "$folder")" == "kernel" && "$systemConf" != "" ]]; then
      # check if directory "$installDir/kernel/systemConfs" exist and create it if not
      if [[ ! -d "$installDir/kernel/systemConfs" ]]; then
        mkdir -p "$installDir/kernel/systemConfs"
      fi
      cp -f "$systemConf" "$installDir/kernel/systemConfs/systemConf.json"
    fi
  done

  # Change the ownership of the directory to the current user
  # Ownership is required before running the service, as the service are run with current user, however $hreFolderPath is owned by root
  chown -R "$currentUser":"$currentUser" "$hreFolderPath"

  echo "Installing kernel executables"
  case "$sdeMode" in
    "u"|"upload-only")
      addExecutable "heexDataUploader" "heexDataUploader" "unknown" "$installDir"

      kernelInstalledList+=("heexDataUploader")
      ;;
    "n"|"no-connectivity")
      addExecutable "heexCore" "heexCore" "unknown" "$installDir"
      addExecutable "heexDataCollector" "heexDataCollector" "unknown" "$installDir"

      kernelInstalledList+=("heexCore")
      kernelInstalledList+=("heexDataCollector")
      ;;
    "l"|"limited-connectivity")
      addExecutable "heexCore" "heexCore" "unknown" "$installDir"
      addExecutable "heexDataCollector" "heexDataCollector" "unknown" "$installDir"
      addExecutable "heexDataUploader" "heexDataUploader" "unknown" "$installDir" "--metadata-only"

      kernelInstalledList+=("heexCore")
      kernelInstalledList+=("heexDataCollector")
      kernelInstalledList+=("heexDataUploader")
      ;;
    *)
      addExecutable "heexCore" "heexCore" "unknown" "$installDir"
      addExecutable "heexDataCollector" "heexDataCollector" "unknown" "$installDir"
      addExecutable "heexDataUploader" "heexDataUploader" "unknown" "$installDir"

      kernelInstalledList+=("heexCore")
      kernelInstalledList+=("heexDataCollector")
      kernelInstalledList+=("heexDataUploader")
      ;;
  esac
}

function installHRE ()
{
  local extractionDir="$1"
  local installDir="$2"
  local installType="$3"

  mkdir -p "$installDir"
  # Own as user this folder. TODO handle when an option is provided to the client to install inside another folder
  chown -R "$currentUser":"$currentUser" "$hreDefaultInstallDirectory"

  if [[ $? -ne 0 ]]; then
    toDisplay="The creation of the installer directory failed. "
    echo "$toDisplay"
    exitBehaviour "false";
  fi

  installKernel "$PWD" "$installDir" "$installType"
  installAgents "$PWD" "$installDir" "$installType"
}

# Compute HRE install type
function computeHREInstallType ()
{
  # HRE version format is x.y.z, where x is the major version, y is the minor and z is the hotfix version
  # Agents are comptabile with kernel where only hte hotfix version changes
  local extractionDir="$1"
  local installDir="$2"
  local __result=$3
  local installType=""

  local currentKernelVersion=""

  local newKernelVersion=$(cat "$extractionDir/version")

  if [[ -f "$installDir/version" ]]
  then
    currentKernelVersion=$(cat "$installDir/version")
  else
    currentKernelVersion="$unknownKernelVersion"
  fi

  if [[ "$currentKernelVersion" == "$newKernelVersion" ]]; then
    installType="install agents"
  else
    currentKernelVersionParts=( ${currentKernelVersion//./ })
    newKernelVersionParts=( ${newKernelVersion//./ })
    if [[ ${currentKernelVersionParts[0]} == ${newKernelVersionParts[0]} && ${currentKernelVersionParts[1]} == ${newKernelVersionParts[1]} ]]; then
      installType="hotfix install"
    else
      installType="fresh install"
    fi
  fi
  eval $__result="'$installType'"
}

# Uninstall HRE
function uninstallHRE ()
{
  local success="true"
  local installDir="$1"

  echo "Uninstalling HRE..."
  local servicesToCheck=()
  # since the installation could have been either for services or background processes, we'll simply try and uninstall both.
  # let's try and uninstall services if ctl can be accessed
  if [[ "$canAccessSystemCtl" == "true" ]]; then
    service_files=$(find /etc/systemd/system -type f -name "*.service")
    # loop through each service file    
    for service_file in $service_files; do
      # leave the heexHreInstallerService file if we are in a ota-update uninstallation
      filename_without_extension=$(basename "$service_file" .service)
      if [[ "$filename_without_extension" == "heexHreInstallerService" && "$installViaOta" == "true" ]]; then
        continue
      fi
      # extract the ExecStart line from the service file
      exec_path=$(grep "ExecStart" "$service_file" | head -n 1 | cut -d= -f2-)
      if grep -q "$hreFolder" <<< "$exec_path"; then
        # extract the service name from the service file name
        service=$(basename "$service_file")
        service="${service%.service}"
        systemctl stop "$service"
        servicesToCheck+=("$service")
        if [[ $? -ne 0 ]]; then
          success="false"
        fi
        systemctl disable "$service"
        if [[ $? -ne 0 ]]; then
          success="false"
        fi
        rm "$service_file"
        if [[ $? -ne 0 ]]; then
          success="false"
        fi
      fi
    done
    # delete the heexHreInstallerService.service file if it exists and if not in ota-update
    if [[ "$installViaOta" == "false" && -L "/etc/systemd/system/heexHreInstallerService.service" ]]; then
      rm "/etc/systemd/system/heexHreInstallerService.service"
    fi

    systemctl daemon-reload
    systemctl reset-failed
  fi
  local processesToCheck=()  # we store the name of each process we need to kill
  local numPidsPerProcess=() # we store the number of PIDs associated to each process
  local pidsList=()          # the list of all the PIDs
  # let's try and uninstall background processes if there are some present
  if [[ -d "$installDir" ]]; then
    while IFS= read -r pid_file; do
      # Read all PIDs from the file into an array
      mapfile -t pidsFromFile < "$pid_file"
      process="${pid_file%.*}"
      process=$(basename "$process")
      numPidsForThisProcess=0
      for pidToKill in "${pidsFromFile[@]}"; do
        ((numPidsForThisProcess++))
        pidsList+=("$pidToKill")
        kill "$pidToKill" &> /dev/null
      done
      processesToCheck+=("$process")
      numPidsPerProcess+=("$numPidsForThisProcess")
    done < <(find "$installDir" -type f -name "*.pid")
  fi

  echo "Verifying that services / background-processes are properly uninstalled:"

  maxWaitTime=10    # seconds. We allow a total of maxWaitTime seconds for all the agents to be running
  currentWaitTime=0 # seconds.
  if [[ "$canAccessSystemCtl" == "true" ]]; then
    for service in "${servicesToCheck[@]}"
    do
      systemStatus=$(systemctl status "$service" 2>/dev/null)
      while [[ $currentWaitTime -lt $((maxWaitTime * 10)) ]]; do
        # as long as service is still running, we wait
        if [[ -z "$systemStatus" ]]; then
          # service is removed.
          break
        fi
        ((currentWaitTime += 1))
        sleep 0.1
        systemStatus=$(systemctl status "$service" 2>/dev/null)
      done
      if [[ -z "$systemStatus" ]]; then
        printf "${GREEN} ${service} UNINSTALLED ${NOCOLOR}\n"
      else
        printf "${RED} ${service} STILL INSTALLED ${NOCOLOR}\n"
      fi
    done
  fi

  pidIndex=0  # this will go over the pidList
  for ((i=0; i<${#processesToCheck[*]}; i++)); do
    process=${processesToCheck[$i]}
    numPids=${numPidsPerProcess[$i]}
    for ((j=0; j<$numPids; j++)); do
      pid=${pidsList[$pidIndex]}
      while [[ $currentWaitTime -lt $((maxWaitTime * 10)) ]]; do
        # as long as process is still running, we wait
        if [[ ! -e "/proc/$pid" ]]; then
          # process pid doesn't exist anymore.
          break
        fi
        ((currentWaitTime += 1))
        sleep 0.1
      done
      if [[ -e "/proc/$pid" ]]; then
        printf "${RED} ${process} STILL INSTALLED ${NOCOLOR}\n"
      else
        printf "${GREEN} ${process} UNINSTALLED ${NOCOLOR}\n"
      fi
      ((pidIndex++))
    done
done

  # remove the hreOtaInstaller that has been copied outside of hre folder if we are not in ota uninstallation
  hreOtaInstallerPath="$(dirname "$installDir")/hreOtaInstaller"
  if [[ $installViaOta == "false" ]] && [[ -f "$hreOtaInstallerPath" ]]; then
      rm -f "$hreOtaInstallerPath"
  fi

  if [[ -d "$installDir" ]]; then
    rm -rf "$installDir"
  fi
  if [[ $? -ne 0 ]]; then
      success="false"
  fi

  if [[ "$success" == "true" ]]; then
    echo "HRE uninstall successful"
    return 0
  else
    echo "HRE uninstall unsuccessful"
    return 1
  fi
}

# This function is used to save the hash of file_to_hash in output_file
function saveHREHash ()
{
    local hash_executable=$1
    local file_to_hash=$2
    local output_file=$3

    local output_dir=$(dirname "$output_file")
    # Create output directory if it does not exist
    if [[ ! -d "$output_dir" ]]; then
        mkdir -p "$output_dir"
    fi
    
    # Delete output file if it already exists. Adding a check on grand_parent_working_dir as well to hack the pipeline issues
    if [[ -f "$output_file" && -n "$grand_parent_working_dir" ]]; then
        rm "$output_file"
    fi

    # Check if hash executable and file to hash exist
    if [[ ! -f "$hash_executable" ]] || [[ ! -f "$file_to_hash" ]]; then
        echo "Error: hash executable ($hash_executable) or file to hash ($file_to_hash) does not exist"
        return 0
    fi

    local hash_value=$("$hash_executable" "$file_to_hash")
    # Length must be 64 characters for the hash + json format
    if [[ $(expr length "$hash_value") -lt 64 ]]; then
      echo "Error: hash value is too short"
      return 0
    fi

    echo "$hash_value" > "$output_file"
    echo "HRE hash saved in $output_file"

    chown -R "$currentUser":"$currentUser" "$output_dir"

    return 0
}

# This function will save in a hre.yaml file some values needed to be saved across updates (ie service/background, user)
saveHreConfig() {
  local hreConfigFilepath="$1"
  local currentUserConf="$2"
  local executionModeConf="$3"

  # Write the configuration to the file
  cat <<EOF > "$hreConfigFilepath"
currentUserConf: $currentUserConf
executionModeConf: $executionModeConf
EOF

  chown "$currentUserConf" "$hreConfigFilepath"

  echo "Configuration saved to $hreConfigFilepath"
}

# This function will read from hre.yaml file if it exists and update the global variables, and if it doesn't exist, it returns 1
# should be called as follow : 'read -r currentUser executionMode <<< "$(readHreConfig "$hreConfigFilepath")"'
readHreConfig() {
  local hreConfigFilepath="$1"

  # Check if the configuration file exists
  if [[ ! -f "$hreConfigFilepath" ]]; then
      printf "${ORANGE} couldn't find $hreConfigFilepath ${NOCOLOR}\n"
      return 1
  fi

  # Read and parse the configuration values from YAML file
  local currentUserConf=$(awk '/currentUserConf/{print $2}' "$hreConfigFilepath")
  local executionModeConf=$(awk '/executionModeConf/{print $2}' "$hreConfigFilepath")

  # Output both values at once
  echo "$currentUserConf $executionModeConf"

  return 0
}

#
# Verify if the systemctl command is available in the environment. Switch to background-process mode if not.
#
checkSystemCtlAccess

#
# Input options

# Transform long options to short ones
for arg in "$@"; do
  shift
  case "$arg" in
    '--help')                    set -- "$@"  '-h'          ;;
    '--os-name')                 set -- "$@"  '-o'          ;;
    '--os-version')              set -- "$@"  '-v'          ;;
    '--architecture')            set -- "$@"  '-a'          ;;
    '--system-conf')             set -- "$@"  '-c'          ;;
    '--mode')                    set -- "$@"  '-m'          ;;
    '--interactive')             set -- "$@"  '-i'          ;;
    '--uninstall')               set -- "$@"  '-u'          ;;
    '--execution')               set -- "$@"  '-e'          ;;
    '--ota-update')              set -- "$@"  '-z'          ;;  # should not be called manually, only for ota
    '--keep-hre-is-installing')  set -- "$@"  '-k'          ;;  # should not be called manually, only for ota
    *)                           set -- "$@"  "$arg"        ;;
  esac
done

# Parse short options
OPTIND=1
while getopts ":ho:v:a:c:m:iue:zk" opt
do
  case "$opt" in
    'h') print_usage; exit 0 ;;
    'o') installerOsName="$OPTARG";
          # This argument is provided by makeself, and is not intended to be used by the user
          if [[ -z "${installerOsName}" ]]
          then
            echo "The installer OS was not set during the installer creation."
            exitBehaviour "false"
          fi;;
    'v') installerOsVersion="$OPTARG";
          # This argument is provided by makeself, and is not intended to be used by the user
          if [[ -z "${installerOsVersion}" ]]
          then
            echo "The installer OS version was not set during the installer creation."
            exitBehaviour "false"
          fi;;
    'a') installerArchitecture="$OPTARG";
          # This argument is provided by makeself, and is not intended to be used by the user
          if [[ -z "${installerArchitecture}" ]]
          then
            echo "The installer architecture was not set during the installer creation."
            exitBehaviour "false"
          fi;;
    'c') systemConf="$OPTARG";
          if [[ -z "${systemConf}" ]]
          then
            echo "The systemConf was not set while using -c option."
            exitBehaviour "false"
          fi;;
    'm') sdeMode="$OPTARG";
          case "$sdeMode" in
            "u"| "upload-only" | "n" | "no-connectivity" | "l" |"limited-connectivity")
              ;;
            *)
              echo "Invalid argument. Usage: u|upload-only|n|no-connectivity|l|limited-connectivity"
              exitBehaviour "false"
            esac;;
    'i') interactiveInstall="true";;
    'u') uninstall="true";;
    'e') exeMode="$OPTARG";
          case $exeMode in
            'service')
              if [[ "$canAccessSystemCtl" == "true" ]]; then
                executionMode='service'
              else
                printf "${ORANGE}[WARNING] You tried running the processes as a service but it is impossible on current system. They shall be run as background-processes.${NOCOLOR}\n"
                executionMode='background-process'
              fi
              ;;
            'background')
              executionMode='background-process'
              ;;
            *)
              if [[ "$canAccessSystemCtl" == "true" ]]; then
                executionMode='service'
              else
                executionMode='background-process'
              fi
              printf "${ORANGE}[WARNING] Wrong input has been given to the --execution (-e) parameter. Using default: $executionMode.${NOCOLOR}\n"
              ;;
          esac
          ;;
    'z') installViaOta="true";;
    'k') keepHreIsInstallingFile="true";;
    '?') >&2 echo "Unknown argument."; print_usage >&2; exitBehaviour "false";;
  esac
done

if [[ -z "$executionMode" ]]; then
  if [[ "$canAccessSystemCtl" == "true" ]]; then
    executionMode='service'
  else
    executionMode='background-process'
  fi
fi

shift $(expr $OPTIND - 1) # remove options from positional parameters


#
# Update HRE install global variables
#
if [[ -z "$hreFolderPath" ]]
then
  hreFolderPath="$hreDefaultInstallDirectory"
else
  hreFolderPath="$(realpath ${hreFolderPath})"
fi
hreFolderPath="$hreFolderPath/$hreFolder"
hreConfigFile="$hreFolderPath/kernel/hreInstallers/hre.yaml"

if [[ "$uninstall" == "true" ]]; then
  uninstallHRE "$hreFolderPath"
  if [[ $? -eq 0 ]]; then
    exit 0
  else
    exit 1
  fi
fi

#
# Client Header script. Client custom script that is run before installing HRE installer
#
if [[ -e HRE_setup_header.sh ]]
then
  source HRE_setup_header.sh
  if [[ $? != 0 ]]; then
    printf "${RED}[ERROR] Sourcing HRE_setup_header.sh failed, the HRE setup might not execute as you expect${NOCOLOR}\n"
    exitBehaviour "false";
  fi
fi

#
# Check that the requirements are met to install the installer (os version, admin rights...)
#
installEssentials
checkInstallerRequirements

#
# Save the hash of the self extracting archive in hreHash.json
#
saveHREHash "$PWD/kernel/sha3-256sum" "$installer_path" "$hreFolderPath/kernel/hreInstallers/hreHash.json"

#
# create the hreIsInstalling file in the installer folder if it is a manual install (ie: not OTA)
# Also, copy the installer into the hreInstaller folder for backup
#
if [[ "$installViaOta" == "false" ]]; then
  touch "$hreFolderPath/kernel/hreInstallers/hreIsInstalling"
  if [ -f "$installer_path" ]; then
      cp "$installer_path" "$hreFolderPath/kernel/hreInstallers"
  fi
fi

#
# If installation is done via OTA, we need to fetch the previous conf instead of the current one that has been set
#
if [[ "$installViaOta" == "true" ]]; then
  echo "installation done via the OTA Update option. Fetching previous conf..."
  read -r currentUser executionMode <<< "$(readHreConfig "$hreConfigFile")"
  confRead=$?
  if [[ "$confRead" -ne 0 ]]; then
    printf "${ORANGE}[WARNING] Failed to fetch conf from previous install. Using default one.${NOCOLOR}\n"
  fi
fi

#
# Save configuration that shall be used during HRE installation
#
saveHreConfig "$hreConfigFile" "$currentUser" "$executionMode"

#
# Determine and start HRE install type
#
computeHREInstallType "$PWD" "$hreFolderPath" result
echo -e "\nInstalling HRE for a $result"
case "$result" in
  "install agents")
    installAgents "$PWD" "$hreFolderPath" "$result";;
  *)
    installHRE "$PWD" "$hreFolderPath" "$result";;
esac

# Change the ownership of the directory to the current user
chown -R "$currentUser":"$currentUser" "$hreFolderPath"

#
# Verifying Kernel services/background processes status
#
if [[ "$result" != "install agents" ]]; then
  echo -e "\nKernel status : "
  for (( i=0; i<${#kernelInstalledList[*]}; i=i+1 )); do
    checkExecutionStatus "${kernelInstalledList[$i]}" kernelExecStatus "$hreFolderPath" "${kernelInstalledList[$i]}"
    case "$kernelExecStatus" in
      "running")
        printf "${GREEN} ${kernelInstalledList[$i]} RUNNING ${NOCOLOR}\n"
        ;;
      *)
        printf "${RED} ${kernelInstalledList[$i]} NOT RUNNING. Check the install.log file and the agent log for more information. ${NOCOLOR}\n"
        ;;
    esac
  done
fi


#
# Client footer script
#
if [[ -e HRE_setup_footer.sh ]]
then
  source HRE_setup_footer.sh
  if [[ $? != 0 ]]; then
    printf "${RED}[ERROR] Sourcing HRE_setup_footer.sh failed, the HRE setup might not execute as you expect${NOCOLOR}\n"
    exitBehaviour "false";
  fi
fi

echo -e "\nHRE installation completed."
if [[ ! -f "$hreFolderPath/kernel/systemConfs/systemConf.json" ]]
then
  echo "Please add your 'systemConf.json' file in "$hreFolderPath"/kernel/systemConfs"
fi;
exitBehaviour "true";
