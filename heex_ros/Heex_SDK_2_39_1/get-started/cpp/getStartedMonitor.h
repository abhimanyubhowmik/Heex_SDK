#pragma once

#include <string>

#include "BooleanMonitor.h"

namespace Heex
{
namespace GetStarted
{
class GetStartedMonitor : public BooleanMonitor
{
public:
  /// A default constructor with forced initialization values
  GetStartedMonitor(const std::string& uuid, const std::string& serverIp, unsigned int& serverPort);
};
} // namespace GetStarted
} // namespace Heex
