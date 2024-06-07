///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/python.hpp>

/// @brief Guard that will acquire the GIL upon construction, and
///        restore its state upon destruction.
class WithGil
{
public:
  WithGil() { _state = PyGILState_Ensure(); }
  ~WithGil() { PyGILState_Release(_state); }

  WithGil(const WithGil&)            = delete;
  WithGil& operator=(const WithGil&) = delete;

private:
  PyGILState_STATE _state;
};
