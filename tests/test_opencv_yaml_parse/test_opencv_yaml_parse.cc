#include "common_utils/opencv_yaml_parse.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

// Test integer parsing
TEST(YamlParserTest, ParseInteger) {
    std::string config_path = "../config/test_opencv_yaml_parse/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    int camera_fps;
    parser.ParseConfig("camera_fps", camera_fps);
    EXPECT_EQ(camera_fps, 30);
}

// Test vector parsing
TEST(YamlParserTest, ParseVector) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    std::vector<int> camera_resolution(2, 0);
    parser.ParseConfig("camera_resolution", camera_resolution);
    EXPECT_EQ(camera_resolution[0], 1920);
    EXPECT_EQ(camera_resolution[1], 1080);
}

// Test 3x3 matrix parsing
TEST(YamlParserTest, ParseMatrix3d) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    parser.ParseConfig("camera_matrix", camera_matrix);
    EXPECT_DOUBLE_EQ(camera_matrix(0, 0), 1000.0);
    EXPECT_DOUBLE_EQ(camera_matrix(0, 2), 640.0);
    EXPECT_DOUBLE_EQ(camera_matrix(1, 1), 1000.0);
}

// Test 4x4 matrix parsing
TEST(YamlParserTest, ParseMatrix4d) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
    parser.ParseConfig("transform_matrix", transform_matrix);
    EXPECT_DOUBLE_EQ(transform_matrix(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(transform_matrix(3, 3), 1.0);
}

// Test boolean parsing from boolean type
TEST(YamlParserTest, ParseBooleanFromBool) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    bool use_feature_detector = false;
    parser.ParseConfig("use_feature_detector", use_feature_detector);
    EXPECT_TRUE(use_feature_detector);
}

// Test boolean parsing from string
TEST(YamlParserTest, ParseBooleanFromString) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    bool feature_enabled = false;
    parser.ParseConfig("feature_enabled", feature_enabled);
    EXPECT_TRUE(feature_enabled);
}

// Test parsing optional parameter (not required)
TEST(YamlParserTest, ParseOptionalParameter) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    int optional_param = 100;  // Default value
    parser.ParseConfig("non_existent_param", optional_param, false);
    EXPECT_EQ(optional_param, 100);  // Should remain unchanged
}

// Test missing required parameter
TEST(YamlParserTest, MissingRequiredParameter) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    int missing_param;
    parser.ParseConfig("missing_param", missing_param, true);
    EXPECT_FALSE(parser.Successful());
}

// Test incorrect type parsing
TEST(YamlParserTest, IncorrectTypeParsing) {
    std::string config_path = "../config/test_opencv_yaml_parse.yaml";
    YamlParser parser(config_path);

    int incorrect_type;
    parser.ParseConfig("camera_matrix", incorrect_type, true);  // Should fail
    EXPECT_FALSE(parser.Successful());
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
