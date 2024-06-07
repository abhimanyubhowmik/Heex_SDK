# 3rdparty

All you need to build Heex embedded projects.
Run the scripts dedicated to your platform to get the third-party resources required by the Heex SDK.

## Windows Platform 3rd-party deployment
Run the scripts to individually download and setup the compilation environment within the ``Windows Powershell`` command prompt: 
- ``getBoost.ps1``: Get 1.75.0 boost header-only libraries and compile the necessary ones as static libs
- ``getCatch.ps1``: Get Catch2 header-only libraries (used and required for tests).
- A ``getEssentials.ps1`` will be provided in a future release to get essential tools to setup build environment for Heex packages and install dependencies on Windows.

If you meet any issue running the ``Windows Powershell`` scripts, you can either consider temporarily authorizing Powershell script execution on your system with ``Set-ExecutionPolicy`` (as ``RemoteSigned`` or  ``Unrestricted``) or running the commands by hand.

## Posix Linux Platform 3rd-party deployment
Run the scripts to individually download and setup the compilation environment within the ``Terminal`` command prompt:
- ``getEssentials.sh``: Get essential tools to setup build environment for Heex packages and install dependencies
- ``getBoost.sh``: Get 1.75.0 boost header-only libraries and compile the necessary ones as static libs
- ``getCatch.sh``: Get Catch2 header-only libraries (used and required for tests).
