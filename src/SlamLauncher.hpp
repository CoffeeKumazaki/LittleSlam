#pragma once

#include "SensorDataReader.hpp"

class SlamLauncher {

public:
  SlamLauncher();
  ~SlamLauncher();

private:
  int startN;


public:
  void run();

private:
  SensorDataReader *sReader;

};

