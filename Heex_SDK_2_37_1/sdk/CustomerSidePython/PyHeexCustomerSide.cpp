///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/bind/bind.hpp>
#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#elif defined(_WIN32) || defined(_WIN64)
  #pragma warning(push)
  #pragma warning(disable : 4267)
#endif
#include <boost/noncopyable.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic pop
#elif defined(_WIN32) || defined(_WIN64)
  #pragma warning(pop)
#endif
#include <vector>

// Customerside headers
#include "Agent.h"
#include "BaseMonitor.h"
#include "BooleanMonitor.h"
#include "InstantMonitor.h"
#include "IntervalMonitor.h"
#include "Monitor.h"
#include "Recorder.h"
#include "StringMonitor.h"
#include "ThresholdMonitor.h"
#include "ZoneMonitor.h"

// CustomerSide headers for backward compatibility
#include "BooleanDetector.h"
#include "InstantDetector.h"
#include "IntervalDetector.h"
#include "StringDetector.h"
#include "ThresholdDetector.h"
#include "ZoneDetector.h"

// Wrapper headers
#include "MonitorWrapper.h"
#include "RecorderWrap.h"

// Utilitary headers that we need
#include "RecorderArgs.h"
#include "ValueConfiguration.h"

template <typename ValueType> boost::python::list convertVectorToList(const std::vector<ValueType>& values)
{
  boost::python::list pyList;

  for (const auto& value : values)
  {
    pyList.append(value);
  }

  return pyList;
}

template <typename KeyType, typename ValueType> boost::python::dict convertMapToDict(const std::unordered_map<KeyType, std::vector<ValueType>>& map)
{
  boost::python::dict pyDict;

  for (const auto& pair : map)
  {
    const KeyType& key                   = pair.first;
    const std::vector<ValueType>& values = pair.second;

    pyDict[key] = convertVectorToList(values);
  }

  return pyDict;
}

BOOST_PYTHON_MODULE(PyHeexCustomerSide)
{
  Py_Initialize();

// Set version for the PyHeexCustomerSide module
#ifndef HEEX_BUILD_VERSION
  #pragma error "Build version not set, using default value"
#endif
  boost::python::scope().attr("__version__") = Heex::HEEX_SYSTEM_SDK_VERSION;

  // Add signal handler enabling SIGINT signal escalation to the C++
  boost::python::exec("import signal; signal.signal(signal.SIGINT, signal.SIG_DFL)");

  // Agent
  boost::python::class_<Agent>("Agent", boost::python::init<const std::string&, const std::string&, const unsigned int&, const std::string&>())
      /// constructor without version
      .def(boost::python::init<const std::string&, const std::string&, const unsigned int&>())
      /// constructor using the configurationFile
      .def(boost::python::init<const std::string&, const std::string&>())
      .def("isConnected", &Agent::isConnected)
      .def("getUuid", &Agent::getUuid)
      .def("getTimestampStr", &Agent::getTimestampStr)
      .def("isReady", &Agent::isReady)
      .def("awaitReady", &Agent::awaitReady)
      .def("reportIncident", &Agent::reportIncident)
      .def("getImplementationVersion", &Agent::getImplementationVersion)
      .def("getSdkVersion", &Agent::getSdkVersion)
      .def("disableLogToFile", &Agent::disableLogToFile)
      .def("getValueConfigurations", &Agent::getValueConfigurations)
      .def(
          "getConstantValues",
          +[](Agent& agent)
          {
            const std::unordered_map<std::string, std::vector<std::string>>& map = agent.getConstantValues();
            return convertMapToDict(map);
          });

  // MonitorWrap
  boost::python::class_<Monitor, boost::noncopyable, boost::python::bases<Agent>>(
      "Monitor", boost::python::init<const std::string&, const std::string&, const unsigned int&, const std::string&>())
      /// constructor without version
      .def(boost::python::init<const std::string&, const std::string&, const unsigned int&>())
      /// constructor using the configurationFile
      .def(boost::python::init<const std::string&, const std::string&>());

  // Setting up Boost Python bindings for BaseMonitor
  void (BaseMonitor::*const signalOn1)(void)                                                   = &BaseMonitor::signalOn;
  void (BaseMonitor::*const signalOn2)(const std::string& timestamp)                           = &BaseMonitor::signalOn;
  void (BaseMonitor::*const signalOn3)(const std::string& timestamp, const std::string& uuid)  = &BaseMonitor::signalOn;
  void (BaseMonitor::*const signalOff1)(void)                                                  = &BaseMonitor::signalOff;
  void (BaseMonitor::*const signalOff2)(const std::string& timestamp)                          = &BaseMonitor::signalOff;
  void (BaseMonitor::*const signalOff3)(const std::string& timestamp, const std::string& uuid) = &BaseMonitor::signalOff;

  /// BaseMonitorWrap is of type boost::python::class_<MonitorWrapper<BaseMonitor>, boost::noncopyable, boost::python::bases<Agent>>
  auto BaseMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<BaseMonitor>, Agent>("BaseMonitor");
  BaseMonitorWrap.def("signalOn", signalOn1);
  BaseMonitorWrap.def("signalOn", signalOn2);
  BaseMonitorWrap.def("signalOn", signalOn3);
  BaseMonitorWrap.def("signalOff", signalOff1);
  BaseMonitorWrap.def("signalOff", signalOff2);
  BaseMonitorWrap.def("signalOff", signalOff3);

  // Setting up Boost Python bindings for BooleanMonitor
  void (BooleanMonitor::*const updateValueBoolean1)(bool inputValue)                               = &BooleanMonitor::updateValue;
  void (BooleanMonitor::*const updateValueBoolean2)(bool inputValue, const std::string& timestamp) = &BooleanMonitor::updateValue;

  /// BooleanMonitorWrap is of type boost::python::class_<MonitorWrapper<BooleanMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto BooleanMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<BooleanMonitor>, BaseMonitor>("BooleanMonitor");
  BooleanMonitorWrap.def("updateValue", updateValueBoolean1);
  BooleanMonitorWrap.def("updateValue", updateValueBoolean2);

  /// BooleanDetectorWrap is of type boost::python::class_<MonitorWrapper<BooleanDetector>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto BooleanDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<BooleanDetector>, BaseMonitor>("BooleanDetector");
  BooleanDetectorWrap.def("updateValue", updateValueBoolean1);
  BooleanDetectorWrap.def("updateValue", updateValueBoolean2);

  // Setting up Boost Python bindings for InstantMonitor
  void (InstantMonitor::*const signalOnOff1)(void)                                                  = &InstantMonitor::signalOnOff;
  void (InstantMonitor::*const signalOnOff2)(const std::string& timestamp)                          = &InstantMonitor::signalOnOff;
  void (InstantMonitor::*const signalOnOff3)(const std::string& timestamp, const std::string& uuid) = &InstantMonitor::signalOnOff;

  /// InstantMonitorWrap is of type boost::python::class_<MonitorWrapper<InstantMonitor>, boost::noncopyable, boost::python::bases<Agent>>
  auto InstantMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<InstantMonitor>, Agent>("InstantMonitor");
  InstantMonitorWrap.def("signalOnOff", signalOnOff1);
  InstantMonitorWrap.def("signalOnOff", signalOnOff2);
  InstantMonitorWrap.def("signalOnOff", signalOnOff3);

  /// InstantDetectorWrap is of type boost::python::class_<MonitorWrapper<InstantMonitor>, boost::noncopyable, boost::python::bases<Agent>>
  auto InstantDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<InstantDetector>, Agent>("InstantDetector");
  InstantDetectorWrap.def("signalOnOff", signalOnOff1);
  InstantDetectorWrap.def("signalOnOff", signalOnOff2);
  InstantDetectorWrap.def("signalOnOff", signalOnOff3);

  // Setting up Boost Python bindings for IntervalMonitor
  void (IntervalMonitor::*const updateValueInterval1)(double inputValue)                               = &IntervalMonitor::updateValue;
  void (IntervalMonitor::*const updateValueInterval2)(double inputValue, const std::string& timestamp) = &IntervalMonitor::updateValue;

  /// IntervalMonitorWrap is of type boost::python::class_<MonitorWrapper<IntervalMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto IntervalMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<IntervalMonitor>, BaseMonitor>("IntervalMonitor");
  IntervalMonitorWrap.def("updateValue", updateValueInterval1);
  IntervalMonitorWrap.def("updateValue", updateValueInterval2);
  IntervalMonitorWrap.def("setSignalUnit", &IntervalMonitor::setSignalUnit);
  IntervalMonitorWrap.def("getSignalUnit", &IntervalMonitor::getSignalUnit);

  /// IntervalDetectorWrap is of type boost::python::class_<MonitorWrapper<IntervalDetector>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto IntervalDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<IntervalDetector>, BaseMonitor>("IntervalDetector");
  IntervalDetectorWrap.def("updateValue", updateValueInterval1);
  IntervalDetectorWrap.def("updateValue", updateValueInterval2);
  IntervalDetectorWrap.def("setSignalUnit", &IntervalMonitor::setSignalUnit);
  IntervalDetectorWrap.def("getSignalUnit", &IntervalMonitor::getSignalUnit);

  /// ThresholdMonitorWrap is of type boost::python::class_<MonitorWrapper<ThresholdMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto ThresholdMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<ThresholdMonitor>, BaseMonitor>("ThresholdMonitor");
  ThresholdMonitorWrap.def("updateValue", updateValueInterval1);
  ThresholdMonitorWrap.def("updateValue", updateValueInterval2);

  /// ThresholdDetectorWrap is of type boost::python::class_<MonitorWrapper<ThresholdDetector>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto ThresholdDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<ThresholdDetector>, BaseMonitor>("ThresholdDetector");
  ThresholdDetectorWrap.def("updateValue", updateValueInterval1);
  ThresholdDetectorWrap.def("updateValue", updateValueInterval2);

  // Setting up Boost Python bindings for StringMonitor
  void (StringMonitor::*const updateValueString1)(std::string inputValue)                               = &StringMonitor::updateValue;
  void (StringMonitor::*const updateValueString2)(std::string inputValue, const std::string& timestamp) = &StringMonitor::updateValue;

  /// StringMonitorWrap is of type boost::python::class_<MonitorWrapper<StringMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto StringMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<StringMonitor>, BaseMonitor>("StringMonitor");
  StringMonitorWrap.def("updateValue", updateValueString1);
  StringMonitorWrap.def("updateValue", updateValueString2);

  /// StringDetectorWrap is of type boost::python::class_<MonitorWrapper<StringDetector>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto StringDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<StringDetector>, BaseMonitor>("StringDetector");
  StringDetectorWrap.def("updateValue", updateValueString1);
  StringDetectorWrap.def("updateValue", updateValueString2);

#ifndef HEEX_SDK_NO_BOOST_JSON
  // Setting up Boost Python bindings for ZoneMonitor
  void (ZoneMonitor::*const updateValueZone1)(double latitude, double longitude)                               = &ZoneMonitor::updateValue;
  void (ZoneMonitor::*const updateValueZone2)(double latitude, double longitude, const std::string& timestamp) = &ZoneMonitor::updateValue;

  /// ZoneMonitorWrap is of type boost::python::class_<MonitorWrapper<ZoneMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto ZoneMonitorWrap = setupMonitorBasicBindings<MonitorWrapper<ZoneMonitor>, BaseMonitor>("ZoneMonitor");
  ZoneMonitorWrap.def("updateValue", updateValueZone1);
  ZoneMonitorWrap.def("updateValue", updateValueZone2);

  /// ZoneMonitorWrap is of type boost::python::class_<MonitorWrapper<ZoneMonitor>, boost::noncopyable, boost::python::bases<BaseMonitor>>
  auto ZoneDetectorWrap = setupMonitorBasicBindings<MonitorWrapper<ZoneDetector>, BaseMonitor>("ZoneDetector");
  ZoneDetectorWrap.def("updateValue", updateValueZone1);
  ZoneDetectorWrap.def("updateValue", updateValueZone2);
#endif

  // RecorderWrap
  boost::python::class_<RecorderWrap, boost::noncopyable, boost::python::bases<Agent>>(
      "Recorder", boost::python::init<const std::string&, const std::string&, const unsigned int&, const std::string&>())
      /// constructor without version
      .def(boost::python::init<const std::string&, const std::string&, const unsigned int&>())
      .def("generateRequestedFilePaths", &RecorderWrap::generateRequestedFilePaths)
      .def("generateRequestedValues", &RecorderWrap::generateRequestedValues)
      .def("addContextValue", &RecorderWrap::addContextValue)
      .def("addGNSSContextValue", &RecorderWrap::addGNSSContextValue)
      .def("getContextValueKeys", &RecorderWrap::getContextValueKeys)
      .def("isPerformingCallback", &Recorder::isPerformingCallback)
      .def("setRealRecordIntervalRange", &RecorderWrap::setRealRecordIntervalRange)
      .def("getEventRecordingPartAnswer", &RecorderWrap::getEventRecordingPartAnswer);

  // RecorderEventRecordingPartArgs
  // Exposing the RecorderEventRecordingPartArgs struct will enable passing the struct as an argument to python
  boost::python::class_<Heex::RecorderArgs::RecorderEventRecordingPartArgs>("RecorderEventRecordingPartArgs", boost::python::init<>())
      .def_readonly("uuid", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::uuid)
      .def_readonly("eventUuid", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::eventUuid)
      .def_readonly("timestamp", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::timestamp)
      .def_readonly("recordIntervalStart", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::recordIntervalStart)
      .def_readonly("recordIntervalEnd", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::recordIntervalEnd)
      .def_readonly("value", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::value)
      .def_readonly("unparsedArgs", &Heex::RecorderArgs::RecorderEventRecordingPartArgs::unparsedArgs);

  // RecorderContextValueArgs
  // Exposing the RecorderContextValueArgs struct will enable passing the struct as an argument to python
  boost::python::class_<Heex::RecorderArgs::RecorderContextValueArgs>("RecorderContextValueArgs", boost::python::init<>())
      .def_readonly("valid", &Heex::RecorderArgs::RecorderContextValueArgs::valid)
      .def_readonly("uuid", &Heex::RecorderArgs::RecorderContextValueArgs::uuid)
      .def_readonly("eventUuid", &Heex::RecorderArgs::RecorderContextValueArgs::eventUuid)
      .def_readonly("timestamp", &Heex::RecorderArgs::RecorderContextValueArgs::timestamp)
      .def_readonly("unparsedArgs", &Heex::RecorderArgs::RecorderContextValueArgs::unparsedArgs)
      .def_readonly("contextValues", &Heex::RecorderArgs::RecorderContextValueArgs::contextValues);

  // ContextValue structure
  boost::python::class_<Heex::RecorderArgs::ContextValue>("ContextValue", boost::python::init<std::string, std::string>())
      .def_readonly("key", &Heex::RecorderArgs::ContextValue::key)
      .def_readonly("value", &Heex::RecorderArgs::ContextValue::value);

  // ValueConfHeader
  boost::python::class_<ValueConfHeader>("ValueConfHeader", boost::python::init<>())
      .def_readonly("uuid", &ValueConfHeader::uuid)
      .def_readonly("cvUuid", &ValueConfHeader::cvUuid)
      .def_readonly("name", &ValueConfHeader::name)
      .def_readonly("params", &ValueConfHeader::params)
      .def_readonly("type", &ValueConfHeader::type)
      .def_readonly("valid", &ValueConfHeader::valid);

  // ValueConfiguration
  boost::python::class_<ValueConfiguration, ValueConfiguration*>("ValueConfiguration", boost::python::init<const ValueConfHeader&>())
      .def("parseStrValue", &ValueConfiguration::parseStrValue)
      .staticmethod("parseStrValue")
      .def("isValid", &ValueConfiguration::isValid, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getUuid", &ValueConfiguration::getUuid, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getCvUuid", &ValueConfiguration::getCvUuid, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getName", &ValueConfiguration::getName, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getType", &ValueConfiguration::getType, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getHeader", &ValueConfiguration::getHeader);

  // ValueConfigurationDouble
  boost::python::class_<ValueConfigurationDouble, boost::python::bases<ValueConfiguration>>("ValueConfigurationDouble", boost::python::init<const ValueConfHeader&>())
      .def("getValue", &ValueConfigurationDouble::getValue);

  // ValueConfigurationString
  boost::python::class_<ValueConfigurationString, boost::python::bases<ValueConfiguration>>("ValueConfigurationString", boost::python::init<const ValueConfHeader&>())
      .def("getValues", &ValueConfigurationString::getValues, boost::python::return_value_policy<boost::python::copy_const_reference>());

  // ValueConfigurationDoubleInterval
  boost::python::class_<ValueConfigurationDoubleInterval, boost::python::bases<ValueConfiguration>>(
      "ValueConfigurationDoubleInterval", boost::python::init<const ValueConfHeader&>())
      .def("getLowValue", &ValueConfigurationDoubleInterval::getLowValue)
      .def("getHighValue", &ValueConfigurationDoubleInterval::getHighValue);

  // ValueConfigurationBoolean
  boost::python::class_<ValueConfigurationBoolean, boost::python::bases<ValueConfiguration>>("ValueConfigurationBoolean", boost::python::init<const ValueConfHeader&>())
      .def("getValue", &ValueConfigurationBoolean::getValue, boost::python::return_value_policy<boost::python::copy_const_reference>());

  // ValueConfigurationThreshold
  boost::python::class_<ValueConfigurationThreshold, boost::python::bases<ValueConfiguration>>("ValueConfigurationThreshold", boost::python::init<const ValueConfHeader&>())
      .def("getValue", &ValueConfigurationThreshold::getValue, boost::python::return_value_policy<boost::python::copy_const_reference>())
      .def("getComparison", &ValueConfigurationThreshold::getComparison, boost::python::return_value_policy<boost::python::copy_const_reference>());

  boost::python::enum_<ValueConfigurationThreshold::Comparison>("Comparison")
      .value("Below", ValueConfigurationThreshold::Above)
      .value("Equal", ValueConfigurationThreshold::Equal)
      .value("Above", ValueConfigurationThreshold::Above);

#ifndef HEEX_SDK_NO_BOOST_JSON
  // ValueConfigurationZone
  boost::python::class_<ValueConfigurationZone, boost::python::bases<ValueConfiguration>>("ValueConfigurationZone", boost::python::init<const ValueConfHeader&>())
      .def("getCircles", &ValueConfigurationZone::getCircles)
      .def("getPolygons", &ValueConfigurationZone::getPolygons);

  // Exposing structures necessary for ValueConfigurationZone
  boost::python::class_<ZoneCircle>("Circle", boost::python::init<double, std::pair<double, double>>())
      .def_readonly("center", &ZoneCircle::center)
      .def_readonly("radius", &ZoneCircle::radius);

  boost::python::class_<ZonePolygon>("Polygon", boost::python::init<std::vector<std::pair<double, double>>>()).def_readonly("points", &ZonePolygon::points);

  boost::python::class_<std::vector<ZoneCircle*>>("stringCircle").def(boost::python::vector_indexing_suite<std::vector<ZoneCircle*>>());
  boost::python::class_<std::vector<ZonePolygon*>>("stringPolygon").def(boost::python::vector_indexing_suite<std::vector<ZonePolygon*>>());
#endif

  // Define a vector of ValueConfiguration* so that python can handle it
  boost::python::class_<std::vector<ValueConfiguration*>>("ValueConfigurationVec").def(boost::python::vector_indexing_suite<std::vector<ValueConfiguration*>>());

  // Define a vector of string
  boost::python::class_<std::vector<std::string>>("stringVec").def(boost::python::vector_indexing_suite<std::vector<std::string>>());

  // Define a vector of ContextValue
  boost::python::class_<std::vector<Heex::RecorderArgs::ContextValue>>("ContextValueVec")
      .def(boost::python::vector_indexing_suite<std::vector<Heex::RecorderArgs::ContextValue>>());
}
