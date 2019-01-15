#ifndef DMP_JOIN_GLOBAL_DEFS_H
#define DMP_JOIN_GLOBAL_DEFS_H

#define PACKAGE_NAME "dmp_join"
// #define CATCH_EXCEPTIONS

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include <armadillo>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

#endif // DMP_JOIN_GLOBAL_DEFS_H
