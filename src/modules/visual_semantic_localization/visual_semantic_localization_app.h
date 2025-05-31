#ifndef VISUAL_SEMANTIC_LOCALIZATION_APP_H
#define VISUAL_SEMANTIC_LOCALIZATION_APP_H

#include <string>
#include <unordered_map>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "visual_semantic_localization_options.h"
#include "data_loader/data_loader.h"
#include "data_types.h"


class VisualSemanticLocalizationApp {
public:
  VisualSemanticLocalizationApp() = default;
  bool Initialize(const std::string &config_yaml_file);
  void Run();

private:
  VisualSemanticLocalizationOptions options_;
  std::shared_ptr<DataLoaderApp> data_loader_;
  
  // Data member variables
  DataLoaderApp::ImageMap raw_image_map_;                           // Raw image data
  DataLoaderApp::ImageMap semantic_mask_image_map_;                 // Semantic mask image data
  std::map<double, IMUData> imu_data_map_;                         // IMU data
  std::map<double, PoseData> ground_truth_map_;                    // Ground truth data
  std::map<double, SemanticContoursData> semantic_contours_map_;   // Semantic contours data
  
  // Data group processing
  std::queue<DataGroup> data_group_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  
  // Data loading functions
  bool LoadAllData();
  bool LoadRawImageData();
  bool LoadSemanticMaskImageData();
  bool LoadIMUData();
  bool LoadGroundTruthData();
  bool LoadSemanticContoursData();
  
  // Data group processing functions
  bool PackDataGroup(const double &timestamp, DataGroup &data_group) ;
  void ProcessDataGroup(const DataGroup& data_group);
  void CleanupOldData();
  
  // Data lookup helper functions
  IMUData* FindClosestIMUData(double timestamp, double tolerance = 0.1);
  PoseData* FindClosestGroundTruthData(double timestamp, double tolerance = 0.1);
  cv::Mat* FindCorrespondingRawImage(double timestamp);
  cv::Mat* FindCorrespondingSemanticImage(double timestamp);
};

#endif // VISUAL_SEMANTIC_LOCALIZATION_APP_H
