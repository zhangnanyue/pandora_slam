#ifndef VISUAL_SEMANTIC_LOCALIZATION_CORE_H
#define VISUAL_SEMANTIC_LOCALIZATION_CORE_H

#include <string>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#include "data_types.h"
#include "visual_semantic_localization_options.h"
#include "ESKF/eskf.h"

class VisualSemanticLocalizationCore {
public:
    VisualSemanticLocalizationCore() = default;
    ~VisualSemanticLocalizationCore() = default;

    bool Initialize(const VisualSemanticLocalizationOptions& options);
    void Run(const DataGroup &data_group);

private:
    bool initialized_ = false;
    bool is_first_frame_ = true;
    double last_frame_timestamp_ = 0.0;
    double current_frame_timestamp_ = 0.0;
    double current_frame_time_diff_ = 0.0;

    
    VisualSemanticLocalizationOptions options_;
    ESKF::Ptr eskf_;
};

#endif