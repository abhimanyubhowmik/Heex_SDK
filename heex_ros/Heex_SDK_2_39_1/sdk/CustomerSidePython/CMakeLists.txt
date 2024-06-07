# Generate and link PyHeexCustomerSide module
add_library(PyHeexCustomerSide MODULE PyHeexCustomerSide.cpp RecorderWrap.cpp) 
target_link_libraries(PyHeexCustomerSide PRIVATE HeexCustomerSide::HeexCustomerSide)
target_link_libraries(PyHeexCustomerSide PRIVATE ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(PyHeexCustomerSide PRIVATE ${PYTHON_INCLUDE_DIRS})
target_include_directories(PyHeexCustomerSide PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(PyHeexCustomerSide PRIVATE -DBOOST_NO_AUTO_PTR)
# Prevent warnings of boost bind placeholders (_1, _2, ...) from the boost library below
target_compile_definitions(PyHeexCustomerSide PRIVATE -DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# Set PyHeexCustomerSide Python library name and extension
if(CMAKE_COMPILER_IS_GNUCXX)
  # Linux platforms
  set_target_properties(PyHeexCustomerSide PROPERTIES POSITION_INDEPENDENT_CODE ON) # Set fPIC flag
  set_target_properties(PyHeexCustomerSide PROPERTIES SUFFIX .so)
  set_target_properties(PyHeexCustomerSide PROPERTIES PREFIX "")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  #Boost.Python is an exception to Boost_USE_STATIC_LIBS, so we need to specify that Boost.Python should be built statically
  target_compile_definitions(PyHeexCustomerSide PRIVATE -DBOOST_PYTHON_STATIC_LIB -DHAVE_SNPRINTF)
  # Support Windows 7 or later
  target_compile_definitions(PyHeexCustomerSide PRIVATE -D_WIN32_WINNT=0x0601)
  # Windows Platforms with MSVC compiler. Python on Windows expects pyd and not dll
  set_target_properties(PyHeexCustomerSide PROPERTIES SUFFIX ".pyd")
endif()
