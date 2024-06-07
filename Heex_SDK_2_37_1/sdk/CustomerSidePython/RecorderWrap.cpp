///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "RecorderWrap.h"

#include "with_gil.h"

RecorderWrap::RecorderWrap(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion)
    : Recorder(uuid, serverIp, serverPort, implementationVersion)
{
}

RecorderWrap::~RecorderWrap() = default;

bool RecorderWrap::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  // Acquire the GIL and release it upon object destruction
  const with_gil gil;

  if (const boost::python::override f = this->get_override("generateRequestedValues"))
  {
    // Extracting the tuple returned by python
    boost::python::object res = f(query, contextValues);
    const bool output         = boost::python::extract<bool>(res[0]);
    contextValues             = boost::python::extract<std::vector<Heex::RecorderArgs::ContextValue>>(res[1]);
    return output;
  }
  return Recorder::generateRequestedValues(query, contextValues);
}

bool RecorderWrap::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  HEEX_LOG(trace) << "In generateRequestedFilePaths" << std::endl;

  // Acquire the GIL and release it upon object destruction
  const with_gil gil;

  if (const boost::python::override f = this->get_override("generateRequestedFilePaths"))
  {
    bool output = false;
    try
    {
      // Extracting the tuple returned by python
      boost::python::object res = f(query, filepath);
      output                    = boost::python::extract<bool>(res[0]);
      filepath                  = boost::python::extract<std::string>(res[1]);
    }
    catch (const boost::python::error_already_set&)
    {
      HEEX_LOG(error) << "RecorderWrap::generateRequestedFilePaths | Error on the python API side." << std::endl;
      PyErr_Print();
      return false;
    }
    catch (const std::exception& e)
    {
      HEEX_LOG(error) << "RecorderWrap::generateRequestedFilePaths | Exception caught: " << e.what() << std::endl;
      return false;
    }
    return output;
  }
  return Recorder::generateRequestedFilePaths(query, filepath);
}
void RecorderWrap::onConfigurationChangedCallback()
{
  //Acquire the GIL and release it upon object destruction
  const with_gil gil;

  if (const boost::python::override f = this->get_override("onConfigurationChangedCallback"))
  {
    f();
  }
}

ValueConfiguration* RecorderWrap::handleValueConfiguration(const std::string& cmd, ValueConfHeader& header)
{
  // Acquire the GIL and release it upon object destruction
  const with_gil gil;

  // return new ValueConfiguration(ValueConfiguration::parseStrValue(cmd));

  if (const boost::python::override f = this->get_override("handleValueConfiguration"))
  {
    const boost::python::object res = f(cmd, header);
    if (res.is_none() == false)
    {
      ValueConfiguration* resultat = boost::python::extract<ValueConfiguration*>(res);
      ValueConfiguration* newVc    = new ValueConfiguration(*resultat);
      // delete resultat; A REVOIR
      return newVc;
    }
    return nullptr;
  }

  return Recorder::handleValueConfiguration(cmd, header);
}

void RecorderWrap::awaitReady()
{
  /// Release the GIL before calling the sleep thread in awaitReady
  PyThreadState* state = PyEval_SaveThread();
  Agent::awaitReady();
  PyEval_RestoreThread(state); // Reacquire the GIL
};

std::vector<std::string> RecorderWrap::getContextValueKeys(const Heex::RecorderArgs::RecorderContextValueArgs& query)
{
  return Recorder::getContextValueKeys(query);
}

bool RecorderWrap::addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value)
{
  return Recorder::addContextValue(res, key, value);
}

bool RecorderWrap::addGNSSContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const double latitude, const double longitude)
{
  return Recorder::addGNSSContextValue(res, key, latitude, longitude);
}

bool RecorderWrap::setRealRecordIntervalRange(
    const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query,
    const std::string& realRecordIntervalStart,
    const std::string& realRecordIntervalEnd)
{
  return Recorder::setRealRecordIntervalRange(query, realRecordIntervalStart, realRecordIntervalEnd);
}

const Heex::RecorderArgs::RecorderEventRecordingPartArgs RecorderWrap::getEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query)
{
  return Recorder::getEventRecordingPartAnswer(query);
}
