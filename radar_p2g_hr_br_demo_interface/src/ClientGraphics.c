
#include <string.h>
#include <ClientLog.h>
#include <ClientGraphics.h>
#include <math.h>

#ifdef LOG_TOKEN
#undef LOG_TOKEN
#endif

#define LOG_TOKEN Module_Graphics

#define PLOT_POLAR_COMMAND_LINES         "plot '-' u 1:2 w p pt 7 ps 3"
#define PLOT_2D_COMMAND_LINES            "plot [%f:%f] [%f:%f] '-' using 1:2 w l dt 1 lw 2 lt 1"
#define REPLOT_2D_COMMAND_LINES          "replot [%f:%f] [%f:%f] '-' using 1:2 w l dt 1 lw 2"
#define PLOT_2D_COMMAND_POINTS           "plot [%f:%f] [%f:%f] '-' using 1:2 with points"
#define PLOT_2D_COMMAND_LINES_FREE_AXIS  "plot [%f:%f] [*:*] '-' using 1:2 w l dt 1 lw 2"
#define PLOT_2D_COMMAND_POINTS_FREE_AXIS "plot [%f:%f] [*:*] '-' using 1:2 with points"
#define PLOT_3D_COMMAND_HEATMAP          "plot [%f:%f] [%f:%f] '-' using 1:2:3 with image"
#define PLOT_3D_COMMAND_MESH             "splot '-' "

static bool polarPlotInitialized = false;
static PlotInstance_t* plotInstanceTable = NULL;
static PlotInstance_t* getPlotInstance(const char* plotStrId)
{
	// If there is no node in the table
	if (plotInstanceTable == NULL)
	{
		return NULL;
	}
	else
	{
		// if there are nodes in the table, if desired
		// node is present already
		PlotInstance_t* currInstance = plotInstanceTable;
		while (currInstance != NULL)
		{
			if (strcmp(currInstance->plotStringId, plotStrId) == 0)
			{
				return currInstance;
			}
			// move to next instance
			currInstance = currInstance->next;
		}
		return NULL;
	}
}

static PlotInstance_t* createPlotInstance(const char* plotStrId)
{
	// Check if instance is already greated
	PlotInstance_t* instance = getPlotInstance(plotStrId);
	PlotInstance_t* currInstance = NULL;

	if (instance != NULL)
	{
		return instance;
	} 
	else
	{
		currInstance = (PlotInstance_t*)malloc(sizeof(PlotInstance_t));
		currInstance->plotHandle = _popen("gnuplot", "w");

		if (currInstance->plotHandle)
		{
			fprintf(currInstance->plotHandle, "set term wxt %d noraise\n");
			currInstance->plotLegendPosition = PLOT_LEGEND_POSITION_INVALID;
			currInstance->plot2DType = PLOT_TYPE_INVALID;
			currInstance->next = NULL;
			strcpy(currInstance->plotStringId, plotStrId);

			// TODO: fill all elements of instance structure

			// It's first elements of instance list
			if (plotInstanceTable == NULL)
			{
				plotInstanceTable = currInstance;
			}
			else
			{
				PlotInstance_t* nextInstance = plotInstanceTable;
				while (nextInstance->next != NULL)
				{
					nextInstance = nextInstance->next;
				}
				nextInstance->next = currInstance;
			}
			return currInstance;
		}
		else
		{
			free(currInstance);
			CLIENT_LOG_FATAL(LOG_TOKEN, "Unable to open gnuplot. Make sure that it's path is set in environment.");
			return NULL; // May not be called depending upon implementation of CLIENT_LOG_FATAL
		}

	}
}

void clientGraphicsPlotPolarPoints(const char* plotId, void* data, int32_t dataLen, const char* legendDesc)
{
	PlotInstance_t* plotInstance = createPlotInstance(plotId);
	if (plotInstance != NULL)
	{
		float r = 0.0;
		float* ptr = data;
		for (int dIdx = 0; dIdx < dataLen; dIdx++)
		{
			if (ptr[dIdx * 2 + 1] > r)
			{
				r = ptr[dIdx * 2 + 1];
			}
		}
		r = ceil(2*r)/2;

		// Prepare title command as part of same command used for plot
		const char gnuPlottitleString[PLOT_CHAR_LEN];
		const char rStr[PLOT_CHAR_LEN];
		sprintf(rStr, "r = %f\n", r);
		sprintf(gnuPlottitleString, "%s title \"%s\"\n", PLOT_POLAR_COMMAND_LINES, legendDesc);
		if (polarPlotInitialized == false)
		{
			//fprintf(plotInstance->plotHandle, "reset\n");
			fprintf(plotInstance->plotHandle, "set polar\n");
			fprintf(plotInstance->plotHandle, "set angle degrees\n");
			fprintf(plotInstance->plotHandle, "set size ratio 1\n");
			fprintf(plotInstance->plotHandle, "set rmargin 6\n");
			fprintf(plotInstance->plotHandle, "set lmargin 6\n");
			fprintf(plotInstance->plotHandle, "set tmargin 4\n");
			fprintf(plotInstance->plotHandle, "set bmargin 4\n");
			fprintf(plotInstance->plotHandle, "set style line 10 lc rgb 'gray80' lt -1\n");
			fprintf(plotInstance->plotHandle, "set grid polar ls 2 dt 1 lw 2\n");
			fprintf(plotInstance->plotHandle, "unset border\n");
			fprintf(plotInstance->plotHandle, "unset tics\n");
			fprintf(plotInstance->plotHandle, rStr);
			fprintf(plotInstance->plotHandle, "set xrange[-r:r]\n");
			fprintf(plotInstance->plotHandle, "set yrange[0:r]\n");
			fprintf(plotInstance->plotHandle, "set size ratio 0.5\n");
			fprintf(plotInstance->plotHandle, "set xtics 0,0.5,r scale 0\n");
			fprintf(plotInstance->plotHandle, "set xtics font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "set rtics 0.5 format '' scale 0\n");
			fprintf(plotInstance->plotHandle, "set label '0°' center at first 0, first r * 1.04 font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "set label '-90°' right at first -r * 1.02, 0 font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "set label '+90°' left at first r * 1.02, 0 font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "unset raxis\n");
			fprintf(plotInstance->plotHandle, "set key outside top right\n");
			polarPlotInitialized = true;
		}
		fprintf(plotInstance->plotHandle, gnuPlottitleString);

		// TODO: only float type is supported
		for (int dIdx = 0; dIdx < dataLen; dIdx++)
		{
			fprintf(plotInstance->plotHandle, "%g %g\n", (double)ptr[dIdx * 2 + 0] + 90, (double)ptr[dIdx * 2 + 1]);
		}
		fprintf(plotInstance->plotHandle, "e\n");

		if (polarPlotInitialized == false)
		{
			fprintf(plotInstance->plotHandle, "set key font \"Sans,15\"\n");
			fprintf(plotInstance->plotHandle, "set xtics font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "set title font \"Sans, 12\"\n");
			fprintf(plotInstance->plotHandle, "set xlabel left font \"Sans, 12\"\n");
		}
		fflush(plotInstance->plotHandle);
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "No plot instance could be created.");
	}
}

void clientGraphicsPlot2DData(const char* plotId, const char* dataType, void* data, int32_t dataLen, bool useLines, 
	const char* legendDesc, PlotAxisData_t* xAxisVal, PlotAxisData_t* yAxisVal, int32_t* xPoints, int32_t lenXPoints, char(*xPointsDesc)[64])
{
	PlotInstance_t* plotInstance = createPlotInstance(plotId);

	if (plotInstance != NULL)
	{
		// Prepare title command as part of same command used for plot
		const char gnuPlottitleString[PLOT_CHAR_LEN];

		// Send command to gnuplot window
		if (lenXPoints == 1)
		{
			char label1Str[128];
			char label1StrCmd[128];
			sprintf(label1Str, "%s%2.1f BPM", xPointsDesc[0], (double)xAxisVal->delta * (double)xPoints[0]);
			sprintf(label1StrCmd, "set label 1 \"%s\" at %f,%f centre font \"Helvetica Bold, 10\"\n", label1Str, (double)xAxisVal->delta * (double)xPoints[0], 1.2);
			fprintf(plotInstance->plotHandle, "unset label 2\n");
			fprintf(plotInstance->plotHandle, label1StrCmd);

			if (strcmp("HR ", xPointsDesc[0]) == 0)
			{
				sprintf(gnuPlottitleString, "%s title \"%s\", '-' w l dt 2 lt 10 lw 2 t \"\"\n",
					useLines ? (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_LINES : PLOT_2D_COMMAND_LINES_FREE_AXIS) :
					           (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_POINTS : PLOT_2D_COMMAND_POINTS_FREE_AXIS), legendDesc);
			}
			else if (strcmp("BR ", xPointsDesc[0]) == 0)
			{
				sprintf(gnuPlottitleString, "%s title \"%s\", '-' w l dt 2 lt 14 lw 2 t \"\"\n",
					useLines ? (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_LINES : PLOT_2D_COMMAND_LINES_FREE_AXIS) :
					           (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_POINTS : PLOT_2D_COMMAND_POINTS_FREE_AXIS), legendDesc);
			}

		}
		else if (lenXPoints == 2)
		{
			char label1Str[128];
			char label2Str[128];
			char label1StrCmd[128];
			char label2StrCmd[128];
			sprintf(label1Str, "%s%2.1f BPM", xPointsDesc[0], (double)xAxisVal->delta * (double)xPoints[0]);
			sprintf(label2Str, "%s%2.1f BPM", xPointsDesc[1], (double)xAxisVal->delta * (double)xPoints[1]);
			sprintf(label1StrCmd, "set label 1 \"%s\" at %f,%f centre font \"Helvetica Bold, 10\"\n", label1Str, (double)xAxisVal->delta * (double)xPoints[0], 1.2);
			sprintf(label2StrCmd, "set label 2 \"%s\" at %f,%f centre font \"Helvetica Bold, 10\"\n", label2Str,(double)xAxisVal->delta * (double)xPoints[1], 1.2);

			fprintf(plotInstance->plotHandle, "unset label 1\n");
			fprintf(plotInstance->plotHandle, "unset label 2\n");
			fprintf(plotInstance->plotHandle, label1StrCmd);
			fprintf(plotInstance->plotHandle, label2StrCmd);

			sprintf(gnuPlottitleString, "%s title \"%s\", '-' w l dt 2 lt 14 lw 2 t '', '-' w l dt 2 lt 10 lw 2 t ''\n",
				useLines ? (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_LINES : PLOT_2D_COMMAND_LINES_FREE_AXIS) :
				(yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_POINTS : PLOT_2D_COMMAND_POINTS_FREE_AXIS), legendDesc);
		}
		else
		{
			sprintf(gnuPlottitleString, "%s title \"%s\"\n", useLines ? (yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_LINES : PLOT_2D_COMMAND_LINES_FREE_AXIS) :
				(yAxisVal->delta != -1.0 ? PLOT_2D_COMMAND_POINTS : PLOT_2D_COMMAND_POINTS_FREE_AXIS), legendDesc);
			fprintf(plotInstance->plotHandle, "unset label 1\n");
			fprintf(plotInstance->plotHandle, "unset label 2\n");
		}
		fprintf(plotInstance->plotHandle, gnuPlottitleString, xAxisVal->start, xAxisVal->end, yAxisVal->start, yAxisVal->end);

		// Send data to gnuplot
		if (strcmp(dataType, "char") == 0)
		{
			char* ptr = data;
			for (int dIdx = 0; dIdx < dataLen; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", (double)(xAxisVal->start + xAxisVal->delta * dIdx), (double)ptr[dIdx]);
			}
		}
		else if (strcmp(dataType, "short") == 0)
		{
			short* ptr = data;
			for (int dIdx = 0; dIdx < dataLen; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", (double)(xAxisVal->start + xAxisVal->delta * dIdx), (double)ptr[dIdx]);
			}
		}
		else if (strcmp(dataType, "int") == 0)
		{
			int* ptr = data;
			for (int dIdx = 0; dIdx < dataLen; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", (double)(xAxisVal->start + xAxisVal->delta * dIdx), (double)ptr[dIdx]);
			}
		}
		else if (strcmp(dataType, "float") == 0)
		{
			float* ptr = data;
			double xAxisTick = (double)xAxisVal->start;
			for (int dIdx = 0; dIdx < dataLen; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", xAxisTick, (double)ptr[dIdx]);
				xAxisTick += xAxisVal->delta;
			}
		}
		else if (strcmp(dataType, "double") == 0)
		{
			double* ptr = data;
			for (int dIdx = 0; dIdx < dataLen; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", (double)(xAxisVal->start + xAxisVal->delta * dIdx), (double)ptr[dIdx]);
			}
		}
		fprintf(plotInstance->plotHandle, "e\n");

		if (lenXPoints)
		{
			for (int dIdx = 0; dIdx < 5; dIdx++)
			{
				fprintf(plotInstance->plotHandle, "%g %g\n", (double)xAxisVal->delta * (double)xPoints[0], (double)dIdx/4.0);
			}
			fprintf(plotInstance->plotHandle, "e\n");

			if (lenXPoints == 2)
			{
				for (int dIdx = 0; dIdx < 5; dIdx++)
				{
					fprintf(plotInstance->plotHandle, "%g %g\n", (double)xAxisVal->delta * (double)xPoints[1], (double)dIdx / 4.0);
				}
				fprintf(plotInstance->plotHandle, "e\n");
			}
		}

		fprintf(plotInstance->plotHandle, "set key font \"Sans,15\"\n");
		fprintf(plotInstance->plotHandle, "set tics font \"Sans, 12\"\n");
		fprintf(plotInstance->plotHandle, "set title font \"Sans, 12\"\n");
		fprintf(plotInstance->plotHandle, "set xlabel font \"Sans, 12\"\n");
		fprintf(plotInstance->plotHandle, "set ylabel font \"Sans, 12\"\n");

		// fprintf(plotInstance->plotHandle, "set arrow from %g,0.0 to %g,1.0 nohead lc rgb 'red' dt 2 t ''\n", (double)xAxisVal->delta * (double)xPoints[0];
		// fprintf(plotInstance->plotHandle, "set arrow from %g,0.0 to %d,1.0 nohead lc rgb 'red' dt 2 t ''\n", (double)xAxisVal->delta * (double)xPoints[1]);

		fflush(plotInstance->plotHandle);
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "No plot instance could be created.");
	}
}

void clientGraphicsPlot3DData(const char* plotId, const char * dataType, void* data, int32_t dataRowLen,  int32_t numRows, 
							   bool useHeatmap, const char* legendDesc, PlotAxisData_t * xAxisVal, PlotAxisData_t *yAxisVal
)
{
	PlotInstance_t* plotInstance = createPlotInstance(plotId);
	if (plotInstance != NULL)
	{
		// Prepare title command as part of same command used for plot
		char gnuPlotTitleString   [PLOT_CHAR_LEN] = "";
		char gnuPlotCommandString [PLOT_CHAR_LEN] = "";
		sprintf(gnuPlotCommandString, useHeatmap ? PLOT_3D_COMMAND_HEATMAP : PLOT_3D_COMMAND_MESH, xAxisVal->start, xAxisVal->end, yAxisVal->start, yAxisVal->end);
		sprintf(gnuPlotTitleString, "%s title \"%s\"\n", gnuPlotCommandString, legendDesc);

		// Send command to gnuplot window
		fprintf(plotInstance->plotHandle, gnuPlotTitleString);
		fflush(plotInstance->plotHandle);

		// Send data to gnuplot
		int sIdx = 0;
		for (int rIdx = 0; rIdx < numRows; rIdx++)
		{
			for (int dIdx = 0; dIdx < dataRowLen; dIdx++)
			{
				int idx = dataRowLen * rIdx + dIdx;

				if (strcmp(dataType, "char") == 0)
				{
					char* ptr = data;
					fprintf(plotInstance->plotHandle, 
						"%g %g %g\n", 
						(double)(xAxisVal->start + xAxisVal->delta * dIdx),
						(double)(yAxisVal->start + yAxisVal->delta * rIdx),
						(double)ptr[idx]);
				}
				else if (strcmp(dataType, "short") == 0)
				{
					short* ptr = data;
					fprintf(plotInstance->plotHandle,
						"%g %g %g\n",
						(double)(xAxisVal->start + xAxisVal->delta * dIdx),
						(double)(yAxisVal->start + yAxisVal->delta * rIdx),
						(double)ptr[idx]);
				}
				else if (strcmp(dataType, "int") == 0)
				{
					int* ptr = data;
					fprintf(plotInstance->plotHandle,
						"%g %g %g\n",
						(double)(xAxisVal->start + xAxisVal->delta * dIdx),
						(double)(yAxisVal->start + yAxisVal->delta * rIdx),
						(double)ptr[idx]);
				}
				else if (strcmp(dataType, "float") == 0)
				{
					float* ptr = data;
					fprintf(plotInstance->plotHandle,
						"%g %g %g\n",
						(double)(xAxisVal->start + xAxisVal->delta * dIdx),
						(double)(yAxisVal->start + yAxisVal->delta * rIdx),
						(double)ptr[idx]);
				}
				else if (strcmp(dataType, "double") == 0)
				{
					double* ptr = data;
					fprintf(plotInstance->plotHandle,
						"%g %g %g\n",
						(double)(xAxisVal->start + xAxisVal->delta * dIdx),
						(double)(yAxisVal->start + yAxisVal->delta * rIdx),
						(double)ptr[idx]);
				}
			}
			fprintf(plotInstance->plotHandle, "\n");
		}
		fprintf(plotInstance->plotHandle, "e\n");
		fflush(plotInstance->plotHandle);
	}
	else
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "No plot instance could be created.");
	}
}
void clientGraphicsSetXrange(const char* plotId, float startXValue, float endXValue)
{
	PlotInstance_t *instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "set xrange [%g:%g]\n", (double)startXValue, (double)endXValue);
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}
void clientGraphicsSetYrange(const char* plotId, float startYValue, float endYValue)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "set yrange [%g:%g]\n", (double)startYValue, (double)endYValue);
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}
void clientGraphicsSetXlabel(const char* plotId, const char* xLabel)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "set xlabel \"%s\"\n", xLabel);
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}
void clientGraphicsSetYlabel(const char* plotId, const char* yLabel)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "set ylabel \"%s\"\n", yLabel);
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGraphicsSetPlotTitle(const char* plotId, const char* plotTitle)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "set title \"%s\"\n", plotTitle);
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGrahicsSetFigureTitle(const char* plotId, const char* figureTitle)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		// TODO: MABD
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGraphicsSetLineWidth(const char* plotId, PlotLineWidth_t lineWidth)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		// TODO: MABD
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGraphicsSetLineMarker(const char* plotId, PlotLineMarker_t marker)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		// TODO: MABD
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGraphicsSetLineStyle(const char* plotId, PlotLineStyle_t style)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		// TODO: MABD
		/*
			 set style line <index> {{linetype  | lt} <line_type> | <colorspec>}
							{{linecolor | lc} <colorspec>}
							{{linewidth | lw} <line_width>}
							{{pointtype | pt} <point_type>}
							{{pointsize | ps} <point_size>}
		*/
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void clientGraphicsSetLineColor(const char* plotId, PlotLineColor_t color)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		// TODO: MABD
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}

void static clientGraphicsCloseFiguresHelper(PlotInstance_t* plotInstance)
{
	if (plotInstanceTable->next->next == NULL)
	{
		free(plotInstanceTable->next);
		plotInstanceTable->next = NULL;
	}
	else
	{
		clientGraphicsCloseFiguresHelper(plotInstanceTable->next);
	}

	// Free head node
	free(plotInstanceTable);

}

void clientGraphicsCloseFigures()
{
	PlotInstance_t* instance;
	while (plotInstanceTable != NULL)
	{
		// close gnuplot window
		_pclose(plotInstanceTable->plotHandle);

		// free the reserved memory for current plot instance
		instance = plotInstanceTable->next;
		free(plotInstanceTable);

		plotInstanceTable = instance;
	}
	plotInstanceTable = NULL;

	if (plotInstanceTable != NULL)
	{
		CLIENT_LOG_FATAL(LOG_TOKEN, "Plot windows could not be closed properly.");
	}
}

void clientGraphicsReplot(const char* plotId)
{
	PlotInstance_t* instance = getPlotInstance(plotId);
	if (instance != NULL)
	{
		fprintf(instance->plotHandle, "replot\n");
		fflush(instance->plotHandle);
	}
	else
	{
		CLIENT_LOG_WARNING(LOG_TOKEN, "Couldn't find a valid plot against plotId. Make sure to issue a plot function before");
	}
}