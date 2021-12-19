#ifndef __CLIENT_LOG_H__
#define __CLIENT_LOG_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define DEBUG_STRING_LEN 200

#define INFO(msg) \
    fprintf(stderr, "info: %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, "%s", msg);

#define eXPAND(token) #token

#define CLIENT_LOG_PRINT(token,fmt,...)   { char debugString[DEBUG_STRING_LEN];\
                                            sprintf(debugString, fmt, ##__VA_ARGS__);\
                                            fprintf(stderr, "%s_Print::%s\n",  eXPAND(token), debugString); \
                                          }

#define CLIENT_LOG_WARNING(token,fmt,...) { char debugString[DEBUG_STRING_LEN];\
                                            sprintf(debugString, fmt, ##__VA_ARGS__);\
                                            fprintf(stderr, "%s_Warning::%s File:%s, Function:%s(), Line:%d\n",  eXPAND(token), debugString, __FILENAME__, __FUNCTION__, __LINE__); \
                                          }

#define CLIENT_LOG_FATAL(token,fmt,...)   { char debugString[DEBUG_STRING_LEN];\
                                            sprintf(debugString, fmt, ##__VA_ARGS__);\
                                            fprintf(stderr, "%s_Fatal::%s...\n  File:%s, Function:%s(), Line:%d\n",  eXPAND(token), debugString, __FILENAME__, __FUNCTION__, __LINE__); \
                                            exit(-1); }



#endif //__CLIENT_LOG_H__
