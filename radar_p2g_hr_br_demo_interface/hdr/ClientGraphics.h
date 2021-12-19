#ifndef __CLIENT_GRAPHICS_H__
#define __CLIENT_GRAPHICS_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define toString(x) #x

#define PLOT_CHAR_LEN 256

#define cLIENT_GRAPHICS_PLOT_POLAR_POINTS(data, len, legend) \
clientGraphicsPlotPolarPoints(#data, data, len, legend)

#define cLIENT_GRAPHICS_PLOT_2D_DATA(data, dataType, len, useLines, legend, xrange, yrange, xPoints, lenXPoints, pointDesc) \
clientGraphicsPlot2DData(#data, dataType, data, len, useLines, legend, xrange, yrange, xPoints, lenXPoints, pointDesc)

#define cLIENT_GRAPHICS_PLOT_3D_DATA(data, dataType, rowLen, numRow, useHeatmap, legend, xvals, yvals) \
        clientGraphicsPlot3DData(#data, dataType, data, rowLen, numRow, useHeatmap, legend, xvals, yvals)

#define cLIENT_GRAPHICS_SET_PLOT_TITLE(name, title)   clientGraphicsSetPlotTitle(#name, title);
#define cLIENT_GRAPHICS_SET_X_LABEL(name, title)      clientGraphicsSetXlabel(#name, title);
#define cLIENT_GRAPHICS_SET_Y_LABEL(name, title)      clientGraphicsSetYlabel(#name, title);
#define cLIENT_GRAPHICS_SET_X_RANGE(name, start, end) clientGraphicsSetXrange(#name, start, end);
#define cLIENT_GRAPHICS_SET_Y_RANGE(name, start, end) clientGraphicsSetYrange(#name, start, end);
#define cLIENT_GRAPHICS_APPLY_SETTINGS(name)          clientGraphicsReplot(#name)

typedef enum PlotType2D_e
{
	PLOT_USING_LINES,
	PLOT_USING_POINTS,
	PLOT_TYPE_INVALID
}PlotType2D_t;

typedef enum PlotLegendPosition_e
{
	PLOT_LEGEND_AT_INSIDE_TOP_RIGHT,
	PLOT_LEGEND_AT_INSIDE_TOP_LEFT,
	PLOT_LEGEND_AT_OUTSIDE_TOP_LEFT,
	PLOT_LEGEND_POSITION_INVALID
} PlotLegendPosition_t;

typedef enum PlotLineColor_s
{
	PLOT_LINE_COLOR_BLACK,
	PLOT_LINE_COLOR_BLUE,
	PLOT_LINE_COLOR_RED,
	PLOT_LINE_COLOR_GREEN,
	PLOT_LINE_COLOR_MAGENTA,
	PLOT_LINE_COLOR_BROWN,
	PLOT_LINE_COLOR_DEFAULT = PLOT_LINE_COLOR_BLUE
}PlotLineColor_t;

typedef enum PlotLineWidth_s
{
	PLOT_LINE_WIDTH_ZERO_P_FIVE,
	PLOT_LINE_WIDTH_ONE,
	PLOT_LINE_WIDTH_ONE_P_FIVE,
	PLOT_LINE_WIDTH_TWO,
	PLOT_LINE_WIDTH_TWO_P_FIVE,
	PLOT_LINE_WIDTH_THREE,
	PLOT_LINE_WIDTH_THREE_P_FIVE,
	PLOT_LINE_WIDTH_DEFAULT = PLOT_LINE_WIDTH_ONE,
} PlotLineWidth_t;

typedef enum PlotLineStyle_s
{
	PLOT_LINE_STYLE_SOLID,
	PLOT_LINE_STYLE_DASHED,
	PLOT_LINE_STYLE_DOTTED,
	PLOT_LINE_STYLE_DASH_DOTTED,
	PLOT_LINE_STYLE_DEFAULT = PLOT_LINE_STYLE_SOLID,
} PlotLineStyle_t;

typedef enum PlotLineMarker_s
{
	PLOT_LINE_MARKER_RECT,
	PLOT_LINE_MARKER_TRI,
	PLOT_LINE_MARKER_CIRC,
	PLOT_LINE_MARKER_CROSS,
	PLOT_LINE_MARKER_DEFAULT = PLOT_LINE_MARKER_TRI
} PlotLineMarker_t;

typedef struct
{
	float start; // axis label value for first point in graph
	float delta; // axis label jump from one point to next
	float end;
} PlotAxisData_t;

/** Once plot is issued, name of variable is
* passed as string to plotting function, which is
* then used to link with gnuplot window handle.
* This is done by incrementing plotId and storing
* relevant strings in a table. This table index
* also links towards the PlotInstance_t table. In
* order to get Instance Id from variable name
* string, variable name is matched with key values
* in table and instance at matched index is used
* for plot functions in any calls
*/
typedef struct PlotInstance_s
{
	FILE* plotHandle;
	char plotStringId[PLOT_CHAR_LEN];
	PlotType2D_t plot2DType;
	PlotLegendPosition_t plotLegendPosition;
	PlotLineWidth_t  lineWidht;
	PlotLineMarker_t linerMarker;
	PlotLineStyle_t lineStyle;
	PlotLineColor_t lineColor;
	struct PlotInstance_s* next;
} PlotInstance_t;

void clientGraphicsPlotPolarPoints(const char* plotId, void* data, int32_t dataLen, const char* legendDesc);
void clientGraphicsPlot2DData(const char* plotId, const char* dataType, void *data, int32_t dataLen, 
	bool useLines, const char* legendDesc, PlotAxisData_t* xAxisVal, PlotAxisData_t* yAxisVal,
	int32_t *xPoints, int32_t lenXPoints, char (*xPointsDesc)[64]);
void clientGraphicsPlot3DData(const char* plotId, const char* dataType, void* data, int32_t dataRowLen, int32_t numRows,
	bool useHeatmap, const char* legendDesc, PlotAxisData_t* xAxisVal, PlotAxisData_t* yAxisVal);
void clientGraphicsSetLineWidth(const char* plotId, PlotLineWidth_t lineWidth);
void clientGraphicsSetLineMarker(const char* plotId, PlotLineMarker_t marker);
void clientGraphicsSetLineStyle(const char* plotId, PlotLineStyle_t style);
void clientGraphicsSetLineColor(const char* plotId, PlotLineColor_t color);
void clientGraphicsSetXrange(const char* plotId, float startXValue, float endXValue);
void clientGraphicsSetYrange(const char* plotId, float startYValue, float endYValue);
void clientGraphicsSetXlabel(const char* plotId, const char *xLabel);
void clientGraphicsSetYlabel(const char* plotId, const char* xLabel);
void clientGraphicsSetPlotTitle(const char* plotId, const char* plotTitle);
void clientGrahicsSetFigureTitle(const char* plotId, const char* figureTitle);
void clientGraphicsReplot(const char* plotId);
void clientGraphicsCloseFigures();

#endif  //__CLIENT_GRAPHICS_H__