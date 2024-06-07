///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <boost/python.hpp>

#include "ValueConfiguration.h"
#include "with_gil.h"

/// template to setup any Monitor class bindings.
template <typename ClassType, typename BaseClassType> auto setupMonitorBasicBindings(const char* className)
{
  boost::python::class_<ClassType, boost::noncopyable, boost::python::bases<BaseClassType>> classObj =
      boost::python::class_<ClassType, boost::noncopyable, boost::python::bases<BaseClassType>>(
          className, boost::python::init<const std::string&, const std::string&, const unsigned int&, const std::string&, const bool&>())
          .def(boost::python::init<const std::string&, const std::string&, const unsigned int&, const std::string&>())
          .def(boost::python::init<const std::string&, const std::string&, const unsigned int&>())
          .def("awaitReady", &ClassType::awaitReady);
  return classObj;
}

// clang-format off
template <typename Base> 
class MonitorWrapper : public Base, public boost::python::wrapper<Base>
// clang-format on
{
public:
  MonitorWrapper(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true)
      : Base(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
  {
  }
  ~MonitorWrapper() = default;

  virtual void awaitReady() override
  {
    /// Release the GIL before calling the sleep thread in awaitReady
    PyThreadState* state = PyEval_SaveThread();
    Agent::awaitReady();
    PyEval_RestoreThread(state); // Reacquire the GIL
  };

  virtual ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& header) override
  {
    //Acquire the GIL and release it upon object destruction
    const with_gil gil;

    if (const boost::python::override f = this->get_override("handleValueConfiguration"))
    {
      const boost::python::object res = f(cmd, header);
      if (res.is_none() == false)
      {
        ValueConfiguration* result = boost::python::extract<ValueConfiguration*>(res);
        ValueConfiguration* newVc  = new ValueConfiguration(*result);
        return newVc;
      }
      return nullptr;
    }

    return Base::handleValueConfiguration(cmd, header);
  };

  virtual void onConfigurationChangedCallback() override
  {
    //Acquire the GIL and release it upon object destruction
    const with_gil gil;

    if (const boost::python::override f = this->get_override("onConfigurationChangedCallback"))
    {
      f();
    }
  }
};
