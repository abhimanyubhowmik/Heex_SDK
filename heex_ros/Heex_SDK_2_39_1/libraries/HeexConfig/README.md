# Config lib

This library handle configuration files.

First, applications have to read one or more conf files with readConfFile method.
Then, applications can require configuration with getConf method.

## Configuration file

Files are as follow :

`
*ClassName1
  Key1 Value1
  Key2 Value2 // Some comments
  ...

*ClassName2
  // Some more comments
  Key1 Value3
  ...
`

ClassNames are always preceded by char '*'.
To access value1, application should call getConf with argument "ClassName1/Key1".
To access value2, application should call getConf with argument "ClassName1/Key2".
To access value3, application should call getConf with argument "ClassName2/Key1".

If a key or a class is read more than once in all read conf files, values are overwritten.
