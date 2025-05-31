#ifndef COMMON_UTILS_COLOR_H
#define COMMON_UTILS_COLOR_H

#include <string>

/**
 * @file color.h
 * @brief 定义用于终端输出的颜色宏。
 *
 * 该头文件包含了一系列用于在终端输出文本时设置颜色的宏定义，
 * 包括基本颜色、高亮颜色以及重置颜色。
 */

/**
 * @def RESET
 * @brief 重置终端颜色。
 *
 * 使用该宏可以将终端的文本颜色重置为默认颜色。
 */
#define RESET "\033[0m"

/**
 * @def BLACK
 * @brief 黑色。
 *
 * 设置文本颜色为黑色。
 */
#define BLACK "\033[30m"                /* Black */

/**
 * @def RED
 * @brief 红色。
 *
 * 设置文本颜色为红色。
 */
#define RED "\033[31m"                  /* Red */

/**
 * @def GREEN
 * @brief 绿色。
 *
 * 设置文本颜色为绿色。
 */
#define GREEN "\033[32m"                /* Green */

/**
 * @def YELLOW
 * @brief 黄色。
 *
 * 设置文本颜色为黄色。
 */
#define YELLOW "\033[33m"               /* Yellow */

/**
 * @def BLUE
 * @brief 蓝色。
 *
 * 设置文本颜色为蓝色。
 */
#define BLUE "\033[34m"                 /* Blue */

/**
 * @def MAGENTA
 * @brief 洋红色。
 *
 * 设置文本颜色为洋红色。
 */
#define MAGENTA "\033[35m"              /* Magenta */

/**
 * @def CYAN
 * @brief 青色。
 *
 * 设置文本颜色为青色。
 */
#define CYAN "\033[36m"                 /* Cyan */

/**
 * @def WHITE
 * @brief 白色。
 *
 * 设置文本颜色为白色。
 */
#define WHITE "\033[37m"                /* White */

/**
 * @def REDPURPLE
 * @brief 红紫色。
 *
 * 设置文本颜色为红紫色。
 */
#define REDPURPLE "\033[95m"            /* Red Purple */

/**
 * @def BOLDBLACK
 * @brief 粗体黑色。
 *
 * 设置文本为粗体黑色。
 */
#define BOLDBLACK "\033[1m\033[30m"     /* Bold Black */

/**
 * @def BOLDRED
 * @brief 粗体红色。
 *
 * 设置文本为粗体红色。
 */
#define BOLDRED "\033[1m\033[31m"       /* Bold Red */

/**
 * @def BOLDGREEN
 * @brief 粗体绿色。
 *
 * 设置文本为粗体绿色。
 */
#define BOLDGREEN "\033[1m\033[32m"     /* Bold Green */

/**
 * @def BOLDYELLOW
 * @brief 粗体黄色。
 *
 * 设置文本为粗体黄色。
 */
#define BOLDYELLOW "\033[1m\033[33m"    /* Bold Yellow */

/**
 * @def BOLDBLUE
 * @brief 粗体蓝色。
 *
 * 设置文本为粗体蓝色。
 */
#define BOLDBLUE "\033[1m\033[34m"      /* Bold Blue */

/**
 * @def BOLDMAGENTA
 * @brief 粗体洋红色。
 *
 * 设置文本为粗体洋红色。
 */
#define BOLDMAGENTA "\033[1m\033[35m"   /* Bold Magenta */

/**
 * @def BOLDCYAN
 * @brief 粗体青色。
 *
 * 设置文本为粗体青色。
 */
#define BOLDCYAN "\033[1m\033[36m"      /* Bold Cyan */

/**
 * @def BOLDWHITE
 * @brief 粗体白色。
 *
 * 设置文本为粗体白色。
 */
#define BOLDWHITE "\033[1m\033[37m"     /* Bold White */

/**
 * @def BOLDREDPURPLE
 * @brief 粗体红紫色。
 *
 * 设置文本为粗体红紫色。
 */
#define BOLDREDPURPLE "\033[1m\033[95m" /* Bold Red Purple */

/**
 * @def BOLD(text)
 * @brief 将文本设置为粗体并重置颜色。
 *
 * 使用该宏可以将传入的文本设置为粗体，并在文本结束后重置颜色。
 *
 * @param text 要设置为粗体的文本。
 *
 * @return 包含颜色控制字符的字符串。
 *
 * @note 该宏在使用时需要确保传入的 `text` 是一个字符串字面量或可拼接的字符串。
 *
 * @code
 * std::cout << BOLD("这是粗体文本") << std::endl;
 * @endcode
 */
#define BOLD(text) "\033[1m" text "\033[0m"

#endif // COMMON_UTILS_COLOR_H
