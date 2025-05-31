#include "visual_semantic_localization/visual_semantic_localization_app.h"
#include "common_utils/print.h"
#include <memory>

int main(int argc, char** argv) {
  if (argc < 2) {
    PRINT_ERROR("用法: %s <config.yaml>\n", argv[0]);
    return -1;
  }

  Printer::SetPrintLevel(Printer::PrintLevel::DEBUG);


  std::string config_file = argv[1];
  auto app = std::make_shared<VisualSemanticLocalizationApp>();
  if (!app->Initialize(config_file)) {
    PRINT_ERROR("VisualSemanticLocalizationApp initialization failed!\n");
    return -1;
  }

  PRINT_INFO("VisualSemanticLocalizationApp execution started...\n");
  app->Run();

  return 0;
}
