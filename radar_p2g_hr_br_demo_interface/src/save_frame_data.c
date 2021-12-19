/*****************************************************
INCLUDES
*****************************************************/
#include <string.h>
#include "mat.h"             // for creating mat files to store data
#include "save_frame_data.h"

/*****************************************************
DATA
*****************************************************/
static int sFrameWriteCount = INT_MAX;

static const char* fileName = "p2g_raw_data.mat";
static MATFile* pmat = NULL;

static mxArray* sMatStructPtr = NULL;

/*****************************************************
LOCAL FUNCTIONS
*****************************************************/

/*****************************************************
GLOBAL FUNCTIONS
*****************************************************/

void saveRadarRawData(
  double* data,
  double  dataLen,
  double* targetRange,
  double  frameNum,
  double  totalFrames,
  bool    reset
)
{

  // Check if need to reset the frame counter?
  if (reset)
  {
    // reset counter
    sFrameWriteCount = 0;
  }

  if (sFrameWriteCount < totalFrames)
  {
    pmat = matOpen(fileName, (sFrameWriteCount == 0) ? "w" : "u");
    if (pmat == NULL)
    {
      printf("Error creating file %s\n", fileName);
      printf("(Do you have write permission in this directory?)\n");
      return(EXIT_FAILURE);
    }

    if (sFrameWriteCount == 0)
    {
      // Check if we need to write to file or not?
      // Reset local frame counter when reset is called
        // Create structure array for each of the data fields
      const int numFields = 4;
      const char* fieldNames[] = { "frame_number", "detected_targets", "adc_data_len", "adc_data" };

      mwSize dims[2] = { 1, totalFrames };
      sMatStructPtr = mxCreateStructArray(2, dims, numFields, fieldNames);
    }
    else
    {
      sMatStructPtr = matGetVariable(pmat, "raw_data");
      if (sMatStructPtr == NULL)
      {
        printf("mxArray not found: %s\n", "raw_data");
        return(EXIT_FAILURE);
      }
    }

    mxArray* mxFrameNum = mxCreateDoubleMatrix(1, 1, mxREAL);
    mxArray* mxTargets = mxCreateDoubleMatrix(2, 1, mxREAL);
    mxArray* mxDataLen = mxCreateDoubleMatrix(1, 1, mxREAL);
    mxArray* mxData = mxCreateDoubleMatrix(dataLen, 1, mxREAL);

    mxGetDoubles(mxFrameNum)[0] = frameNum;
    mxGetDoubles(mxTargets)[0] = targetRange[0];
    mxGetDoubles(mxTargets)[1] = targetRange[1];
    mxGetDoubles(mxDataLen)[0] = dataLen;
    memcpy((void*)mxGetDoubles(mxData), (void*)data, dataLen * sizeof(data));

    mxSetFieldByNumber(sMatStructPtr, sFrameWriteCount, 0, mxFrameNum);
    mxSetFieldByNumber(sMatStructPtr, sFrameWriteCount, 1, mxTargets);
    mxSetFieldByNumber(sMatStructPtr, sFrameWriteCount, 2, mxDataLen);
    mxSetFieldByNumber(sMatStructPtr, sFrameWriteCount, 3, mxData);

    // Save structure to MAT file
    int status = matPutVariable(pmat, "raw_data", sMatStructPtr);
    if (status != 0) {
      printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
      return(EXIT_FAILURE);
    }

    mxDestroyArray(mxFrameNum);
    mxDestroyArray(mxTargets);
    mxDestroyArray(mxDataLen);
    mxDestroyArray(mxData);

    // Increment the frame counter
    sFrameWriteCount++;

    if (matClose(pmat) != 0)
    {
      printf("Error closing file %s\n", fileName);
    }
  }
}
