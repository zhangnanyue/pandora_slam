#include "visual_semantic_localization/visual_semantic_localization_app.h"
#include "visual_semantic_localization/visual_semantic_localization_options.h"
#include "common_utils/print.h"
#include <memory>

int main(int argc, char** argv) {
  if (argc < 2) {
    PRINT_ERROR("用法: %s <config.yaml>\n", argv[0]);
    return -1;
  }

  Printer::SetPrintLevel(Printer::PrintLevel::DEBUG);

  // 加载配置
  std::string config_file = argv[1];
  VisualSemanticLocalizationOptions options;
  if (!VisualSemanticLocalizationOptions::LoadFromYaml(config_file, options)) {
    PRINT_ERROR("Failed to load configuration from %s\n", config_file.c_str());
    return -1;
  }

  // 打印配置信息
  options.Print();

  // 创建并初始化应用
  auto app = std::make_shared<VisualSemanticLocalizationApp>();
  if (!app->Initialize(options)) {
    PRINT_ERROR("VisualSemanticLocalizationApp initialization failed!\n");
    return -1;
  }

  PRINT_INFO("VisualSemanticLocalizationApp execution started...\n");
  app->Run();

  return 0;
}
