#include "print.h"

// 需要定义静态变量以使一切正常工作
Printer::PrintLevel Printer::current_print_level_ = PrintLevel::INFO;

void Printer::SetPrintLevel(const std::string &level) {
  if (level == "ALL")
    SetPrintLevel(PrintLevel::ALL);
  else if (level == "DEBUG")
    SetPrintLevel(PrintLevel::DEBUG);
  else if (level == "INFO")
    SetPrintLevel(PrintLevel::INFO);
  else if (level == "WARNING")
    SetPrintLevel(PrintLevel::WARNING);
  else if (level == "ERROR")
    SetPrintLevel(PrintLevel::ERROR);
  else if (level == "SILENT")
    SetPrintLevel(PrintLevel::SILENT);
  else {
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void Printer::SetPrintLevel(PrintLevel level) {
  Printer::current_print_level_ = level;
  std::cout << "Setting printing level to: ";
  switch (current_print_level_) {
  case PrintLevel::ALL:
    std::cout << "ALL";
    break;
  case PrintLevel::DEBUG:
    std::cout << "DEBUG";
    break;
  case PrintLevel::INFO:
    std::cout << "INFO";
    break;
  case PrintLevel::WARNING:
    std::cout << "WARNING";
    break;
  case PrintLevel::ERROR:
    std::cout << "ERROR";
    break;
  case PrintLevel::SILENT:
    std::cout << "SILENT";
    break;
  default:
    std::cout << std::endl;
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << std::endl;
}

std::string Printer::GetCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto milliseconds_since_epoch =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch())
          .count();
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
#if defined(_WIN32) || defined(_WIN64)
  localtime_s(&now_tm, &now_time_t);
#else
  localtime_r(&now_time_t, &now_tm);
#endif
  // 计算当前秒中的毫秒部分
  int milliseconds = milliseconds_since_epoch % 1000;

  // 格式化时间为 [YYYY-MM-DD HH:MM:SS.mmm]
  std::ostringstream oss;
  oss << "[" << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "."
      << std::setfill('0') << std::setw(3) << milliseconds << "]";
  return oss.str();
}

const char *Printer::GetColorCode(PrintLevel level) {
  switch (level) {
    case PrintLevel::ALL:
      return RESET;
    case PrintLevel::DEBUG:
      return BLUE;
    case PrintLevel::INFO:
      return GREEN;
    case PrintLevel::WARNING:
      return YELLOW;
    case PrintLevel::ERROR:
      return RED;
    case PrintLevel::SILENT:
      return RESET;
    default:
      return RESET;
  }
}

void Printer::DebugPrint(PrintLevel level, const char location[],
                         const char line[], const char *format, ...) {
  // Only print for the current debug level
  if (static_cast<int>(level) <
      static_cast<int>(Printer::current_print_level_)) {
    return;
  }

  // 获取当前时间戳
  std::string timestamp = GetCurrentTimestamp();
  printf("%s", timestamp.c_str());

  // Print the location info first for our debug output
  // Truncate the filename to the max size for the filepath
  if (static_cast<int>(Printer::current_print_level_) <=
      static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > MAX_FILE_PATH_LEGTH) {
      printf("[%s", base_filename
                        .substr(base_filename.size() - MAX_FILE_PATH_LEGTH,
                                base_filename.size())
                        .c_str());
    } else {
      printf("[%s", base_filename.c_str());
    }
    printf(":%s] ", line);
  }

  // 添加颜色代码
  printf("%s", GetColorCode(level));

  // Print the rest of the args
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);

  // 重置颜色
  printf("%s", RESET);
}
