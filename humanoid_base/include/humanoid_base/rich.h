#ifndef __RICI_H__
#define __RICI_H__

#define CL_RESET "\033[0m"

#define CL_BLACK "\033[30m"              /* Black */
#define CL_RED "\033[31m"                /* Red */
#define CL_GREEN "\033[32m"              /* Green */
#define CL_YELLOW "\033[33m"             /* Yellow */
#define CL_BLUE "\033[34m"               /* Blue */
#define CL_MAGENTA "\033[35m"            /* Magenta */
#define CL_CYAN "\033[36m"               /* Cyan */
#define CL_WHITE "\033[37m"              /* White */
#define CL_BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define CL_BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define CL_BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define CL_BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define CL_BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define CL_BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define CL_BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define CL_BOLDWHITE "\033[1m\033[37m"   /* Bold White */

#include <string>
#include <memory>
#include <stdexcept>

template <typename... Args>
std::string string_format(const std::string& format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
                 1;  // Extra space for '\0'
    if (size_s <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(),
                       buf.get() + size - 1);  // We don't want the '\0' inside
}

#endif  // __RICI_H__
