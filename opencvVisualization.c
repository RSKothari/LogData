// -----------------------------------------------------------------------
//
// (c) Copyright 1997-2015, SensoMotoric Instruments GmbH
// 
// Permission  is  hereby granted,  free  of  charge,  to any  person  or
// organization  obtaining  a  copy  of  the  software  and  accompanying
// documentation  covered  by  this  license  (the  "Software")  to  use,
// reproduce,  display, distribute, execute,  and transmit  the Software,
// and  to  prepare derivative  works  of  the  Software, and  to  permit
// third-parties to whom the Software  is furnished to do so, all subject
// to the following:
// 
// The  copyright notices  in  the Software  and  this entire  statement,
// including the above license  grant, this restriction and the following
// disclaimer, must be  included in all copies of  the Software, in whole
// or  in part, and  all derivative  works of  the Software,  unless such
// copies   or   derivative   works   are   solely   in   the   form   of
// machine-executable  object   code  generated  by   a  source  language
// processor.
// 
// THE  SOFTWARE IS  PROVIDED  "AS  IS", WITHOUT  WARRANTY  OF ANY  KIND,
// EXPRESS OR  IMPLIED, INCLUDING  BUT NOT LIMITED  TO THE  WARRANTIES OF
// MERCHANTABILITY,   FITNESS  FOR  A   PARTICULAR  PURPOSE,   TITLE  AND
// NON-INFRINGEMENT. IN  NO EVENT SHALL  THE COPYRIGHT HOLDERS  OR ANYONE
// DISTRIBUTING  THE  SOFTWARE  BE   LIABLE  FOR  ANY  DAMAGES  OR  OTHER
// LIABILITY, WHETHER  IN CONTRACT, TORT OR OTHERWISE,  ARISING FROM, OUT
// OF OR IN CONNECTION WITH THE  SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// -----------------------------------------------------------------------

#include "opencvVisualization.h"
#include "main.h"
#include <WinDef.h>
#include <highgui.h>
#include "iostream"

char gLeftEyeImageWindowDisplayed = 0;
char gRightEyeImageWindowDisplayed = 0;
char gSceneImageWindowDisplayed = 0;
static char autoScaleOnce = 1;



/* **************************************************************************************** */

IplImage *scaleImage (IplImage * const img, const float factor) {
	CvSize size = cvSize ((int) (img->width * factor), (int) (img->height * factor));
	int depth = img->depth;
	int channels = img->nChannels;
	IplImage *result = cvCreateImage (size, depth, channels);
	// nearest-neigbor interpolation.
	// Options:
	//   CV_INTER_NN     = nearest neigbor
	//   CV_INTER_LINEAR = linear
	//   CV_INTER_AREA   = pixel area realatioin
	//   CV_INTER_CUBIC  = bicubic
	cvResize (img, result, CV_INTER_CUBIC);
	return result;
	
}

/* **************************************************************************************** */

void onMouseInSceneImage (const int event, const int x, const int y, const int flags, void * param) {

	switch (event) {
		case CV_EVENT_MOUSEMOVE:
			break;

		case CV_EVENT_LBUTTONDOWN:
			break;

		case CV_EVENT_LBUTTONUP:
			removeMouseCallbackOnSceneImageDisplay ();
			Calibrate (x / gScaleScene, y / gScaleScene);
			break;
	}

	return;
}

/* **************************************************************************************** */

void createSceneImageWindow () {
	// if you upscale the scene and have trouble calibrating, try using CV_WINDOW_NORMAL
	// instead of autosize
	cvNamedWindow ("Scene Image", CV_WINDOW_AUTOSIZE);
	gSceneImageWindowDisplayed = 1;
}

/* **************************************************************************************** */

void createLeftEyeImageWindow () {
	cvNamedWindow ("Left Eye Image", CV_WINDOW_AUTOSIZE);
	gLeftEyeImageWindowDisplayed = 1;
}

/* **************************************************************************************** */

void createRightEyeImageWindow () {
	cvNamedWindow ("Right Eye Image", CV_WINDOW_AUTOSIZE);
	gRightEyeImageWindowDisplayed = 1;
}

/* **************************************************************************************** */

void displaySceneImage (iViewImage *image) {
	if (NULL == image)
		return;

	if (0 == gSceneImageWindowDisplayed)
		createSceneImageWindow ();

	// scale window automatically if it does not fit on the screen.
	// but only if nobody set the scene scale manually.
	if (!gScaleSceneSet && autoScaleOnce) {
		autoScaleOnce = 0;
		int borderSizeX = 4*GetSystemMetrics(SM_CXSIZEFRAME);
		int borderSizeY = 4*GetSystemMetrics(SM_CYSIZEFRAME) + GetSystemMetrics(SM_CYCAPTION);
		if(GetSystemMetrics(SM_CXSCREEN) < (image->width + borderSizeX) || GetSystemMetrics(SM_CYSCREEN) < (image->height + borderSizeY)){
			gScaleScene = MIN((float)GetSystemMetrics(SM_CXSCREEN) / (image->width  + borderSizeX),\
							  (float)GetSystemMetrics(SM_CYSCREEN) / (image->height + borderSizeY));
			printf ("\n(autoscaling window to fit the screen)\n");
		}
	}

	IplImage * scaledImage = scaleImage ((IplImage *) image, gScaleScene);

	cvShowImage ("Scene Image", scaledImage);


	if (gCalibrationPointsToDo > 0) {

		// ignore any calibration attempts if not enough time has passed since calibration setup.
		iViewTicket * timeTicket = NULL;
		iView_CreateTicket(&timeTicket);
		iView_GetLocalTime(timeTicket);
		unsigned int timeNowMsec = ((iViewTime *) timeTicket->result->data)->time / 1000000;
		iView_ReleaseTicket(&timeTicket);

		if (timeNowMsec - gTimeOfSetupCalibMsec > gSetupCalibCooldownMsec) {
			// enough time has passed
			setMouseCallbackOnSceneImageDisplay ();
		}
	}

	cvWaitKey (1);
	cvReleaseImage (&scaledImage);

	return;
}

/* **************************************************************************************** */

void drawGazeOverlay (iViewImage * image, const int x, const int y) {

	cvCircle ((IplImage*) image, cvPoint (x, y), 22, cvScalar (0., 0., 255.), -1, CV_AA, 0);
	return;
}

/* **************************************************************************************** */

void displayLeftEyeImage (iViewImage *image) {

	if (NULL == image)
		return;

	if (0 == gLeftEyeImageWindowDisplayed)
		createLeftEyeImageWindow ();

	IplImage * scaledImage = scaleImage ((IplImage *) image, gScaleEyes);

	cvShowImage ("Left Eye Image", scaledImage);

	cvWaitKey (1);
	cvReleaseImage (&scaledImage);

	return;
}

void writeImage (iViewImage *image, int fr_num, string imLoc, vector<int> compression_params){
	
	if (NULL == image)
		return;

	string tempStr = imLoc + "\\" + to_string( static_cast<long double>(fr_num)) + ".jpg";

	cvSaveImage(tempStr.c_str(), image);
	return;
}

/* **************************************************************************************** */

void displayRightEyeImage (iViewImage *image) {

	if (NULL == image)
		return;

	if (0 == gRightEyeImageWindowDisplayed)
		createRightEyeImageWindow ();

	IplImage * scaledImage = scaleImage ((IplImage *) image, gScaleEyes);

	cvShowImage ("Right Eye Image", scaledImage);
	cvWaitKey (1);
	cvReleaseImage (&scaledImage);

	return;
}

/* **************************************************************************************** */

void runImageDisplayEventLoop (int delay) {
	cvWaitKey (delay);
}

/* **************************************************************************************** */

void closeSceneImageDisplay () {
	cvDestroyWindow ("Scene Image");
}

/* **************************************************************************************** */

void closeLeftEyeImageDisplay () {
	if (!gLeftEyeImageWindowDisplayed)
		return;

	cvDestroyWindow ("Left Eye Image");
	gLeftEyeImageWindowDisplayed = 0;
}

/* **************************************************************************************** */

void closeRightEyeImageDisplay () {
	if (!gRightEyeImageWindowDisplayed)
		return;

	cvDestroyWindow ("Right Eye Image");
	gRightEyeImageWindowDisplayed = 0;
}

/* **************************************************************************************** */

void setMouseCallbackOnSceneImageDisplay () {
	cvSetMouseCallback ("Scene Image", onMouseInSceneImage, NULL );
}

/* **************************************************************************************** */

void removeMouseCallbackOnSceneImageDisplay () {
	cvSetMouseCallback ("Scene Image", NULL, NULL );
}

/* **************************************************************************************** */
