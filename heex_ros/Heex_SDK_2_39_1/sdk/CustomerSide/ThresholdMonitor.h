///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
///
/// @file ThresholdMonitor.h
/// @brief Rename former ThresholdMonitor to IntervalMonitor to keep backward compatibility when compiling code still using ThresholdMonitor.
#pragma once

#include "IntervalMonitor.h"

using ThresholdMonitor = IntervalMonitor;
