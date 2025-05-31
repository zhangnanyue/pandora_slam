#include "common_utils/opencv_yaml_parse.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <string>
#include <vector>

int main() {
  // Path to the YAML configuration file
  std::string config_path =
      "/home/zny/projects/pandora_slam/install/config/"
      "opencv_yaml_parse_example/opencv_yaml_parse_example.yaml";

  // Initialize the YamlParser
  YamlParser parser(config_path);

  // Variables to store parsed values
  int max_iterations = 50; // Default value
  double threshold = 0.1;  // Default value
  float pi_value = 3.14f;  // Default value
  std::string greeting = "Default Greeting";

  bool is_enabled = false;
  bool is_visible = true;
  bool use_cache = false;
  bool debug_mode = true;
  bool invalid_bool_test = false; // This should trigger a warning

  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix4d transformation_matrix;

  // Optional parameter
  std::string optional_param = "Default Optional";

  // Parse basic types
  parser.ParseConfig("max_iterations", max_iterations); // Required
  parser.ParseConfig("threshold", threshold);           // Required
  parser.ParseConfig("pi_value", pi_value);             // Required
  parser.ParseConfig("greeting", greeting);             // Required

  // Parse boolean values
  parser.ParseConfig("is_enabled", is_enabled); // Required
  parser.ParseConfig("is_visible", is_visible); // Required
  parser.ParseConfig("use_cache", use_cache);   // Required
  parser.ParseConfig("debug_mode", debug_mode); // Required
  parser.ParseConfig("invalid_bool_test", invalid_bool_test,
                      false); // Optional (invalid)

  // Parse Eigen matrices
  parser.ParseConfig("rotation_matrix", rotation_matrix); // Required
  parser.ParseConfig("transformation_matrix",
                      transformation_matrix); // Required

  // Parse optional parameter
  parser.ParseConfig("optional_param", optional_param, false); // Optional

  // Check if all required parameters were successfully parsed
  if (!parser.Successful()) {
    std::cerr << "Some parameters were not successfully parsed. Please check "
                 "the warnings above.\n";
  }

  // Output parsed values
  std::cout << "Parsed Configuration:\n";
  std::cout << "max_iterations: " << max_iterations << "\n";
  std::cout << "threshold: " << threshold << "\n";
  std::cout << "pi_value: " << pi_value << "\n";
  std::cout << "greeting: " << greeting << "\n\n";

  std::cout << "Boolean Values:\n";
  std::cout << "is_enabled: " << std::boolalpha << is_enabled << "\n";
  std::cout << "is_visible: " << is_visible << "\n";
  std::cout << "use_cache: " << use_cache << "\n";
  std::cout << "debug_mode: " << debug_mode << "\n";
  std::cout << "invalid_bool_test: " << invalid_bool_test << " (Expected: false)\n\n";

  std::cout << "Rotation Matrix:\n" << rotation_matrix << "\n\n";
  std::cout << "Transformation Matrix:\n" << transformation_matrix << "\n\n";

  std::cout << "Optional Parameter:\n" << optional_param << "\n";

  return 0;
}