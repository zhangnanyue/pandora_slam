#include "common_utils/print.h"
#include <string>

int main() {
    // Example file paths (some intentionally long to demonstrate truncation)
    std::string short_file = "main.cpp";
    std::string long_file = "this/is/a/very/long/path/to/the/main.cpp/file/that/needs/truncation.cpp";

    // 1. Setting Print Levels using Enum
    PRINT_INFO("Setting print level to DEBUG using enum.\n");
    Printer::SetPrintLevel(Printer::PrintLevel::DEBUG);
    PRINT_INFO("Current print level is DEBUG.\n");

    // 2. Printing messages at various levels
    PRINT_ALL("This is an ALL level message. It should always appear if level is ALL.\n");
    PRINT_DEBUG("This is a DEBUG level message.\n");
    PRINT_INFO("This is an INFO level message.\n");
    PRINT_WARNING("This is a WARNING level message.\n");
    PRINT_ERROR("This is an ERROR level message.\n");

    // 3. Changing Print Level to INFO using String
    PRINT_INFO("Setting print level to INFO using string.\n");
    Printer::SetPrintLevel("INFO");
    PRINT_INFO("Current print level is INFO.\n");

    // 4. Printing messages after changing print level
    PRINT_ALL("This ALL message should NOT appear when level is INFO.\n");
    PRINT_DEBUG("This DEBUG message should NOT appear when level is INFO.\n");
    PRINT_INFO("This INFO message should appear when level is INFO.\n");
    PRINT_WARNING("This WARNING message should appear when level is INFO.\n");
    PRINT_ERROR("This ERROR message should appear when level is INFO.\n");

    // 5. Changing Print Level to WARNING
    PRINT_INFO("Setting print level to WARNING.\n");
    Printer::SetPrintLevel(Printer::PrintLevel::WARNING);
    PRINT_INFO("Current print level is WARNING.\n");

    // 6. Printing messages after setting to WARNING
    PRINT_ALL("This ALL message should NOT appear when level is WARNING.\n");
    PRINT_DEBUG("This DEBUG message should NOT appear when level is WARNING.\n");
    PRINT_INFO("This INFO message should NOT appear when level is WARNING.\n");
    PRINT_WARNING("This WARNING message should appear when level is WARNING.\n");
    PRINT_ERROR("This ERROR message should appear when level is WARNING.\n");

    // 7. Changing Print Level to ERROR
    PRINT_INFO("Setting print level to ERROR.\n");
    Printer::SetPrintLevel("ERROR");
    PRINT_INFO("Current print level is ERROR.\n");

    // 8. Printing messages after setting to ERROR
    PRINT_ALL("This ALL message should NOT appear when level is ERROR.\n");
    PRINT_DEBUG("This DEBUG message should NOT appear when level is ERROR.\n");
    PRINT_INFO("This INFO message should NOT appear when level is ERROR.\n");
    PRINT_WARNING("This WARNING message should NOT appear when level is ERROR.\n");
    PRINT_ERROR("This ERROR message should appear when level is ERROR.\n");

    // 9. Changing Print Level to SILENT
    PRINT_INFO("Setting print level to SILENT.\n");
    Printer::SetPrintLevel(Printer::PrintLevel::SILENT);
    PRINT_INFO("Current print level is SILENT.\n");

    // 10. Printing messages after setting to SILENT
    PRINT_ALL("This ALL message should NOT appear when level is SILENT.\n");
    PRINT_DEBUG("This DEBUG message should NOT appear when level is SILENT.\n");
    PRINT_INFO("This INFO message should NOT appear when level is SILENT.\n");
    PRINT_WARNING("This WARNING message should NOT appear when level is SILENT.\n");
    PRINT_ERROR("This ERROR message should NOT appear when level is SILENT.\n");

    // // 11. Attempting to set an invalid print level
    // PRINT_INFO("Attempting to set an invalid print level 'VERBOSE'.\n");
    // try {
    //     Printer::SetPrintLevel("VERBOSE");
    // } catch (...) {
    //     PRINT_ERROR("Caught an exception while setting an invalid print level.\n");
    // }

    // 12. Demonstrating file name truncation
    PRINT_DEBUG("\nDemonstrating file name truncation with a long file path.\n");
    // Temporarily set print level to DEBUG to see debug messages
    Printer::SetPrintLevel(Printer::PrintLevel::DEBUG);
    PRINT_DEBUG("Short file path test: %s\n", short_file.c_str());
    PRINT_DEBUG("Long file path test: %s\n", long_file.c_str());

    // Reset print level to INFO
    Printer::SetPrintLevel(Printer::PrintLevel::INFO);

    return 0;
}
