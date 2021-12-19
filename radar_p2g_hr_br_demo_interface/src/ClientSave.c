#include <stdbool.h>
#include <Shlwapi.h>
#include "ClientSave.h"
#include <ClientConfig.h>

#define MAX_NUM_FILES_IN_RECORD 100

static char s_saveDirectoryOutTxtFiles[PATH_MAX_CHAR] = { NULL };
static char s_saveDirectoryInTxtFiles [PATH_MAX_CHAR] = { NULL };

static void clientSaveCleanWriteDirectory()
{
    if (PathIsDirectoryA(s_saveDirectoryOutTxtFiles))
    {
        // create wildcard search file names
        char saveDirectoryTxtFiles[PATH_MAX_CHAR];
        strcpy(saveDirectoryTxtFiles, s_saveDirectoryOutTxtFiles);
        strcat(saveDirectoryTxtFiles, DIR_SEPARATOR);
        strcat(saveDirectoryTxtFiles, "*.txt");

        WCHAR wSearchName[_MAX_PATH];
        // encode string for search
        MultiByteToWideChar(CP_UTF8, 0, saveDirectoryTxtFiles, _MAX_PATH, wSearchName, _MAX_PATH);

        WIN32_FIND_DATA fdata;
        HANDLE fhandle = FindFirstFile(wSearchName, &fdata);

        // delete if a file is found
        while (fhandle != INVALID_HANDLE_VALUE)
        {
            char wFileName[_MAX_PATH];
            WideCharToMultiByte(CP_UTF8, 0, fdata.cFileName, _MAX_PATH, wFileName, _MAX_PATH, '0', FALSE);
            sprintf(saveDirectoryTxtFiles, "%s%s%s",s_saveDirectoryOutTxtFiles, DIR_SEPARATOR, wFileName);

            // delete current file
            if (DeleteFileA(saveDirectoryTxtFiles) == FALSE)
            {
                CLIENT_LOG_FATAL(LOG_TOKEN, "Could not delete file %s from path %s.", wFileName, saveDirectoryTxtFiles);
            }

            // keep moving to next files
            while (FindNextFile(fhandle, &fdata))
            {
                WideCharToMultiByte(CP_UTF8, 0, fdata.cFileName, _MAX_PATH, wFileName, _MAX_PATH, '0', FALSE);
                sprintf(saveDirectoryTxtFiles, "%s%s%s", "clientSaveCleanWriteDirectory", DIR_SEPARATOR, wFileName);

                // delete current file
                if (DeleteFileA(saveDirectoryTxtFiles) == FALSE)
                {
                    CLIENT_LOG_FATAL(LOG_TOKEN, "Could not delete file %s from path %s.", wFileName, saveDirectoryTxtFiles);
                }
            }
            fhandle = INVALID_HANDLE_VALUE;
        }

        // check if directory is cleaned up
        if (PathIsDirectoryEmptyA(s_saveDirectoryOutTxtFiles) == 0)
        {
            CLIENT_LOG_FATAL(LOG_TOKEN, "Directory %s is not empty.", s_saveDirectoryOutTxtFiles);
        }
        else
        {
            CLIENT_LOG_PRINT(LOG_TOKEN, "Directory %s is cleaned.", s_saveDirectoryOutTxtFiles);
        }
    }
    else
    {
        CLIENT_LOG_PRINT(LOG_TOKEN, "Directory %s is not present. Nothing to do.", s_saveDirectoryOutTxtFiles);
    }
}

void clientSaveInit()
{
    char* outDirectoryName = (char*)malloc(PATH_MAX_CHAR * sizeof(char));
    char* inDirectoryName = (char*)malloc(PATH_MAX_CHAR * sizeof(char));

    // Get current working directory and create name for save directory
    DWORD outDirPathLen = GetCurrentDirectoryA(PATH_MAX_CHAR, (char*)outDirectoryName);
    DWORD inDirPathLen  = GetCurrentDirectoryA(PATH_MAX_CHAR, (char*)inDirectoryName);
    
    if (outDirPathLen)
    {
        outDirectoryName = strcat(outDirectoryName, DIR_SEPARATOR);
        outDirectoryName = strcat(outDirectoryName, WRITE_DIR_NAME);

        inDirectoryName = strcat(inDirectoryName, DIR_SEPARATOR);
        inDirectoryName = strcat(inDirectoryName, READ_DIR_NAME);
    }
    else
    {
        CLIENT_LOG_FATAL(LOG_TOKEN, "Current directory is not valid");
    }
    // Save directory name to static variables for further calls
    strcpy(s_saveDirectoryOutTxtFiles, outDirectoryName);
    strcpy(s_saveDirectoryInTxtFiles,  inDirectoryName);

    // create directory if it doesn't already exist
    if (PathIsDirectoryA(s_saveDirectoryOutTxtFiles))
    {
        CLIENT_LOG_PRINT(LOG_TOKEN, "%s is a already present directory", outDirectoryName);
        // clean contents of directory here.
        clientSaveCleanWriteDirectory();
    }
    else
    {
        // directory is not present, let's create a directory here.
        if (CreateDirectoryA(outDirectoryName, NULL) != 0)
        {
            CLIENT_LOG_PRINT(LOG_TOKEN, "Creating directory%s. Directory created.", outDirectoryName);
        }
        else
        {
            CLIENT_LOG_FATAL(LOG_TOKEN, "Directory %s could not be created.", outDirectoryName);
        }
    }

    free(outDirectoryName);

    // Check if input directory is present

    // create directory if it doesn't already exist
    if (PathIsDirectoryA(s_saveDirectoryInTxtFiles) == FALSE)
    {
        // directory is not present, let's create a directory here.
        if (CreateDirectoryA(s_saveDirectoryInTxtFiles, NULL) != 0)
        {
            CLIENT_LOG_PRINT(LOG_TOKEN, "Creating directory%s. Directory created.", s_saveDirectoryInTxtFiles);
        }
        else
        {
            CLIENT_LOG_FATAL(LOG_TOKEN, "Directory %s could not be created.", s_saveDirectoryInTxtFiles);
        }
    }
}

void clientSaveStoreDataTxtFile(char* fileName, const char*dataType, const void* dataPtr, int32_t dataLen, bool isComplex, bool isInOutData)
{
    char fileDirectory[PATH_MAX_CHAR];
    sprintf(fileDirectory, "%s", isInOutData ? s_saveDirectoryInTxtFiles : s_saveDirectoryOutTxtFiles);

    if (PathIsDirectoryA(fileDirectory))
    {
        char fullFilePath[PATH_MAX_CHAR];
        sprintf(fullFilePath, "%s%s%s.txt", fileDirectory, DIR_SEPARATOR, fileName);
        FILE* fileHandle = fopen(fullFilePath, isInOutData ? "w" : "a");

        fprintf(fileHandle, "\n");

        if (strcmp(dataType, "short") == 0)
		{
			short* ptr = dataPtr;
			if (isComplex)
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%d, %d\n", ptr[2 * i], ptr[2 * i + 1]);
				}
			}
			else
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%d\n", ptr[i]);
				}
			}
		}
        else if (strcmp(dataType, "int") == 0)
		{
			int* ptr = dataPtr;
			if (isComplex)
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%d, %d\n", ptr[2 * i], ptr[2 * i + 1]);
				}
			}
			else
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%f\n", ptr[i]);
				}
			}
		}
		else if (strcmp(dataType, "float") == 0)
		{
			float* ptr = dataPtr;
			if (isComplex)
			{
				for (int32_t i = 0; i < dataLen; i++)				{
					fprintf(fileHandle, "%8.4f, %8.4f\n", ptr[2 * i], ptr[2 * i + 1]);
				}
			}
			else
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%8.4f\n", ptr[i]);
				}
			}
		}
		else if (strcmp(dataType, "double") == 0)
		{
			double* ptr = dataPtr;
			if (isComplex)
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%8.4f, %8.4f\n", ptr[2 * i], ptr[2 * i + 1]);
				}
			}
			else
			{
				for (int32_t i = 0; i < dataLen; i++)
				{
					fprintf(fileHandle, "%8.4f\n", ptr[i]);
				}
			}
		}
		fflush(fileHandle);
        fclose(fileHandle);
    }
    else
    {
        CLIENT_LOG_WARNING(LOG_TOKEN, "Save directory should be created first.");
    }
}

bool clientSaveReadCalibrationData(const char* fileName, const char* dataType, void *dataPtr, int32_t dataLen, bool readComplexData)
{
    char fileDirectory[PATH_MAX_CHAR];
    sprintf(fileDirectory, "%s", s_saveDirectoryInTxtFiles);

    if (PathIsDirectoryA(s_saveDirectoryInTxtFiles))
    {
        char fullFilePath[PATH_MAX_CHAR];
        sprintf(fullFilePath, "%s%s%s.txt", fileDirectory, DIR_SEPARATOR, fileName);
        FILE* fileHandle = fopen(fullFilePath, "r");

        if (fileHandle)
		{
			if (strcmp(dataType, "short") == 0)
			{
				short* ptr = dataPtr;
				if (readComplexData)
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%d, %d\n", ptr[2 * i], ptr[2 * i + 1]);
					}
				}
				else
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%d\n", ptr[i]);
					}
				}
			}
			else if (strcmp(dataType, "int") == 0)
			{
				int* ptr = dataPtr;
				if (readComplexData)
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%d, %d\n", ptr[2 * i], ptr[2 * i + 1]);
					}
				}
				else
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%f\n", ptr[i]);
					}
				}
			}
			else if (strcmp(dataType, "float") == 0)
			{
				float* ptr = dataPtr;
				if (readComplexData)
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%f,%f\n", &ptr[2 * i], &ptr[2 * i + 1]);
					}
				}
				else
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%8.4f\n", ptr[i]);
					}
				}
			}
			else if (strcmp(dataType, "double") == 0)
			{
				double* ptr = dataPtr;
				if (readComplexData)
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fscanf(fileHandle, "%8.4f, %8.4f\n", ptr[2 * i], ptr[2 * i + 1]);
					}
				}
				else
				{
					for (int32_t i = 0; i < dataLen; i++)
					{
						fprintf(fileHandle, "%8.4f\n", ptr[i]);
					}
				}
			}
			fclose(fileHandle);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Save directory should be created first.");
	}
}
