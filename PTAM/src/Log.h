#include <stdio.h>

#define LOG_MODE 1

#if LOG_MODE
    #undef LOG
    #define LOG_STREAM stdout
    #define LOG(msg,...) {fprintf(LOG_STREAM, "[%s:%d] %s(): ", __FILE__, __LINE__, __FUNCTION__); fprintf(LOG_STREAM, msg, ##__VA_ARGS__); fprintf(LOG_STREAM, "\n");}
#else
    #undef LOG
    #define LOG(msg,...)
#endif
