///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/python.hpp>

/// @brief Guard that will acquire the GIL upon construction, and
///        restore its state upon destruction.
class with_gil
{
public:
  with_gil() { state_ = PyGILState_Ensure(); }
  ~with_gil() { PyGILState_Release(state_); }

  with_gil(const with_gil&)            = delete;
  with_gil& operator=(const with_gil&) = delete;

private:
  PyGILState_STATE state_;
};
