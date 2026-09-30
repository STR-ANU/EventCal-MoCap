#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "yaml-cpp/yaml.h"
#include <string>

struct Parameters
{
  // General
  bool f_fixed_coordinates;
  int led_px;
  int led_py;
  int led_r;

};

Parameters loadParametersFromYAML(const std::string &yaml_file_path);

#endif // PARAMETERS_H