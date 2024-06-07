/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
// clang-format off
#ifndef APEX_AI_HEEX_DEMO_PKG__VISIBILITY_CONTROL_HPP_
#define APEX_AI_HEEX_DEMO_PKG__VISIBILITY_CONTROL_HPP_

#include <apexutils/apexdef.h>  // for APEX_WINDOWS, APEX_LINUX, APEX_OSX, APEX_QNX

////////////////////////////////////////////////////////////////////////////////
#if defined(APEX_WINDOWS)
#if defined(APEX_AI_HEEX_DEMO_PKG_BUILDING_DLL) || \
    defined(APEX_AI_HEEX_DEMO_PKG_EXPORTS)
#define APEX_AI_HEEX_DEMO_PKG_PUBLIC __declspec(dllexport)
#define APEX_AI_HEEX_DEMO_PKG_LOCAL
#else  // defined(APEX_AI_HEEX_DEMO_PKG_BUILDING_DLL) ||
       // defined(APEX_AI_HEEX_DEMO_PKG_EXPORTS)
#define APEX_AI_HEEX_DEMO_PKG_PUBLIC __declspec(dllimport)
#define APEX_AI_HEEX_DEMO_PKG_LOCAL
#endif  // defined(APEX_AI_HEEX_DEMO_PKG_BUILDING_DLL) ||
        // defined(APEX_AI_HEEX_DEMO_PKG_EXPORTS)
#elif defined(APEX_LINUX)
#define APEX_AI_HEEX_DEMO_PKG_PUBLIC __attribute__((visibility("default")))
#define APEX_AI_HEEX_DEMO_PKG_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_OSX)
#define APEX_AI_HEEX_DEMO_PKG_PUBLIC __attribute__((visibility("default")))
#define APEX_AI_HEEX_DEMO_PKG_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_QNX)
#define APEX_AI_HEEX_DEMO_PKG_PUBLIC __attribute__((visibility("default")))
#define APEX_AI_HEEX_DEMO_PKG_LOCAL __attribute__((visibility("hidden")))
#else  // defined(APEX_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(APEX_WINDOWS)

#endif  // APEX_AI_HEEX_DEMO_PKG__VISIBILITY_CONTROL_HPP_
// clang-format on
