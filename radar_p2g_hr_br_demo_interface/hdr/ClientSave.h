#ifndef __CLIENT_SAVE_H__
#define __CLIENT_SAVE_H__

#include <stdbool.h>
#include <stdint.h>
#include <Windows.h>
#include <sys/stat.h>

#include "ClientLog.h"

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif 
#define LOG_TOKEN Module_Save

#define cLIENT_SAVE_STORE_DATA_TXT_FILE(data, type, len, isCplx, isInOutData) \
  clientSaveStoreDataTxtFile(#data, type, data, len, isCplx, isInOutData);

#define cLIENT_SAVE_READ_DATA_TXT_FILE(fileName, dataType, data, dataLen, readCplxData) \
  clientSaveReadCalibrationData(fileName, dataType, data, dataLen, readCplxData)

#define CLIENT_SAVE_STATUS_OK                            0
#define CLIENT_SAVE_STATUS_ERROR_FILE_NOT_CREATED       -1
#define CLIENT_SAVE_STATUS_ERROR_FILE_NOT_DELETED       -2
#define CLIENT_SAVE_STATUS_ERROR_DIRECTORY_NOT_CREATED  -3
#define CLIENT_SAVE_STATUS_ERROR_DIRECTORY_NOT_DELETED  -4

#define DIR_SEPARATOR  "\\"
#define PATH_MAX_CHAR  250
#define READ_DIR_NAME  "txt_in"
#define WRITE_DIR_NAME "txt_out"

#define sTRINGIZE(token)      #token
#define cONC(prefix,suffix)   sTRINGIZE(prefix##_##suffix)
#define tO_STR(prefix,suffix) cONC(prefix,suffix)

/** This function creates save directory such that 
* txt log files can be placed within. If directory
* already present then, it cleans it's contents such
* that previous txt files can be deleted and new
* data for current run of application can be run.
*/
void clientSaveInit();

/** This function stored data in text file named
* save_<variableName>.txt. It creates file on 
* first call for a particular variable, on 
* second and on call for variable data storage,
* data is appended to that file.
*/
void clientSaveStoreDataTxtFile(char* fileName, const char* dataType, const void* dataPtr, int32_t dataLen, bool isComplex, bool isInOutData);

/** This function read data.
*
*
*/
bool clientSaveReadCalibrationData(const char* fileName, const char* dataType, void* dataPtr, int32_t dataLen, bool readComplexData);

#endif //__CLIENT_SAVE_H__