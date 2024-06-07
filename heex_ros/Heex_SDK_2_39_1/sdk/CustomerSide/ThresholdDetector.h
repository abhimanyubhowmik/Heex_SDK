///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
///
/// @file ThresholdDetector.h
/// @brief Rename former ThresholdDetector to IntervalMonitor to keep backward compatibility when compiling code still using ThresholdDetector.
#pragma once

#include "IntervalMonitor.h"

using ThresholdDetector = IntervalMonitor;
