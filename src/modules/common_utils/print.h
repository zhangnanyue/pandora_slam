#ifndef COMMON_UTILS_PRINT_H
#define COMMON_UTILS_PRINT_H

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include "colors.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>


/**
 * @brief 打印类，允许进行各种级别的打印
 *
 * 设置全局打印级别的方法如下：
 * @code{.cpp}
 * Printer::setPrintLevel("WARNING");
 * Printer::setPrintLevel(Printer::PrintLevel::WARNING);
 * @endcode
 */
class Printer {
public:
  /**
   * @brief 不同的打印级别
   *
   * - PrintLevel::ALL : 所有 PRINT_XXXX 将输出到控制台
   * - PrintLevel::DEBUG : 仅打印 "DEBUG", "INFO", "WARNING" 和 "ERROR"。
   *    不打印 "ALL"
   * - PrintLevel::INFO : 仅打印 "INFO", "WARNING" 和 "ERROR"。
   *    不打印 "ALL" 和 "DEBUG"
   * - PrintLevel::WARNING : 仅打印 "WARNING" 和 "ERROR"。
   *    不打印 "ALL", "DEBUG" 和 "INFO"
   * - PrintLevel::ERROR : 仅打印 "ERROR"。不打印其他所有
   * - PrintLevel::SILENT : 所有 PRINT_XXXX 都将被静音。
   */
  enum PrintLevel {
    ALL = 0,
    DEBUG = 1,
    INFO = 2,
    WARNING = 3,
    ERROR = 4,
    SILENT = 5
  };

  /**
   * @brief 设置用于所有未来打印到 stdout 的打印级别。
   * @param level 要使用的调试级别（字符串形式）
   */
  static void SetPrintLevel(const std::string &level);

  /**
   * @brief 设置用于所有未来打印到 stdout 的打印级别。
   * @param level 要使用的调试级别（枚举形式）
   */
  static void SetPrintLevel(PrintLevel level);

  /**
   * @brief 打印函数，打印到 stdout。
   * @param level 此打印调用的打印级别
   * @param location 打印调用的位置（文件名）
   * @param line 打印调用的行号
   * @param format printf 格式
   */
  static void DebugPrint(PrintLevel level, const char location[],
                         const char line[], const char *format, ...);

  /// 当前的打印级别
  static PrintLevel current_print_level_;

private:
  /// 文件路径的最大长度。这是为了避免非常长的文件路径
  static constexpr uint32_t MAX_FILE_PATH_LEGTH = 40;

  // 根据打印级别返回对应的颜色代码
  static const char *GetColorCode(PrintLevel level);

  // 获取当前时间的字符串表示
  static std::string GetCurrentTimestamp();
};

/*
 * 将任何内容转换为字符串
 */
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/*
 * 不同类型的打印级别
 */
#define PRINT_ALL(x...) Printer::DebugPrint(Printer::PrintLevel::ALL, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_DEBUG(x...) Printer::DebugPrint(Printer::PrintLevel::DEBUG, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_INFO(x...) Printer::DebugPrint(Printer::PrintLevel::INFO, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_WARNING(x...) Printer::DebugPrint(Printer::PrintLevel::WARNING, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_ERROR(x...) Printer::DebugPrint(Printer::PrintLevel::ERROR, __FILE__, TOSTRING(__LINE__), x);

#endif /* COMMON_UTILS_PRINT_H */
