# libraries

Collection of Heex Embedded C++ libraries.

## Libraries compilation
Required dependencies:
- Boost 3rd-party libraries. Clone the `3rdparty` repository are required.
To compile the librairies, execute the command `./bootstrap.sh`. For more options, see the `-h` option.

## HeexCom
This C++ library handles all related client and server sides to operate interprocess connection between agent within the Heex Embedded environment. See all SDK-specific messages in the README file of the SDK.

## HeexConfig
This C++ toolbox parses *Heex-custom format* configuration file to store Heex-specific system configurations. See more details in the dedicated [HeexConfig Readme](./HeexConfig/README.md)

## HeexMessages
This C++ library defines the C++ structures and tools to manage, process and send interprocess messages between SDK and SDE. See more details in the dedicated [HeexMessages Readme](./HeexMessages/README.md)

## HeexRestApiManager
This C++ library wraps the usage of the Heex APIs to upload Smart Data.
Required dependencies:
-  libcurl-dev library. Install it using the following command:
  ```bash
  sudo apt install libcurl4-openssl-dev # libcurl-dev
  ```

## HeexUtils
This C++ toolbox library provides utility operations within the Heex Embedded environment.
