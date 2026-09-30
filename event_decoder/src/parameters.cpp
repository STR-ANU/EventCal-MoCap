#include "parameters.h"

Parameters loadParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    Parameters params;

    // General
    params.f_fixed_coordinates = config["f_fixed_coordinates"].as<bool>();
    params.led_px = config["led_px"].as<int>();
    params.led_py = config["led_py"].as<int>();
    params.led_r = config["led_r"].as<int>();

    return params;
}