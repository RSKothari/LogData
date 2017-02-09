// -----------------------------------------------------------------------
//
// (c) Copyright 1997-2014, SensoMotoric Instruments GmbH
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

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
//to deal with compile error from pthreads (Win 32 version)
#define HAVE_STRUCT_TIMESPEC

#include <opencv.hpp>

#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <direct.h>
#include <stdlib.h>
#include <pthread.h>
#include <iViewNG-Core.h>
#include <iViewNG-Calibration.h>
#include <iViewNG-Connection.h>
#include <iViewNG-DataAcquisition.h>
#include <iViewNG-Device-ETG.h>

#include <iViewNG-Utility.h>

#include "iViewNG-Convenience.h"
#include "main.h"
#include "OpenCvVisualization.h"


/* **************************************************************************************** */
/* *************************************** PROTOTYPES ************************************* */
/* **************************************************************************************** */

iViewRC ParseCommandLine (int, char **);
iViewRC Setup ();
iViewRC SetupCalib ();
iViewRC Subscribe ();
iViewRC Start ();
iViewRC Cleanup ();
iViewRC Calibrate ();
void WaitForUserInteraction ();
void MyCallback (iViewTicket * const ticket);

/* **************************************************************************************** */
/* ***************************************   DATA     ************************************* */
/* **************************************************************************************** */

iViewTicket * gTicketStartAcquisition = NULL;
iViewTicket * gTicketConnect = NULL;
iViewTicket * gTicketAddLicense = NULL;
iViewTicket * gTicketDeviceParameters = NULL;
iViewTicket * gTicketSubscriptionGaze = NULL;
iViewTicket * gTicketSubscriptionLeftEye = NULL;
iViewTicket * gTicketSubscriptionRightEye = NULL;
iViewTicket * gTicketSubscriptionScene = NULL;
iViewTicket * gTicketSubscriptionSceneWithGaze = NULL;
iViewTicket * gTicketSubscriptionSceneH264 = NULL;
iViewTicket * gTicketSubscriptionSceneH264WithGaze = NULL;
iViewTicket * gTicketUnsubscription = NULL;
iViewTicket * gTicketStopAcquisition = NULL;
iViewTicket * gTicketCalibration1Pt = NULL;
iViewTicket * gTicketCalibration3Pt[3];

iViewHost gServer;

// Flag which tells us that the remote iViewNG-Server is used
// in conjunction with iViewETG-Client, so this Tutorial-RemoteViewer
// will not set device parameters (as the parameters set by
// iViewETG-Client would then be overwritten).
char gRemoteIsIviewEtg = 0;

// Flag which specifies whether an explicit shutdown command
// is to be sent to the server when this tutorial ends.
char gShutdownServer = 0;

// User-provided flags
char gShowGaze = 0;
char gShowEyeImages = 0;
char gShowSceneImages = 0;
char gShowSceneH264 = 0;
char gShowSceneImagesWithGaze = 0;
char gShowSceneH264ImagesWithGaze = 0;
char gCalibrate1Pt = 0;
char gCalibrate3Pt = 0;
char gScene24 = 0;
char gScene30 = 0;
char gGazeOverlay = 0;
iViewSamplingRate gSamplingRate = IVIEWSAMPLINGRATE_CURRENT;
char gTimeMaster = 0;
iViewEyeCamExposureMode gEyeExposureMode = IVIEWEYECAMEXPOSUREMODE_CURRENT;

// Scale factors applied to the eye/scene image for downscaling.
float gScaleEyes = 1.;
float gScaleScene = 1.;
char gScaleSceneSet = 0;

const float gSamplerateEyes = 99.;
const float gSamplerateScene = 99.;

// Local data for calibration
char gCalibrationPointsToDo = 0;
unsigned int gCurrentFrameNumber = 0;

// Local data for gaze overlay
int gGazeX = 0;
int gGazeY = 0;

unsigned int gTimeOfSetupCalibMsec = 0;
unsigned int gSetupCalibCooldownMsec = 2000;

const unsigned int TICKET_WAIT_MS = 5000;
using namespace cv;

vector<int> compression_params;

// File handling: Rakshit
std::ofstream GazeFile;
char str2Write[1000];

// Image writeout parameters: Rakshit
string RightEyeImageLoc;
string LeftEyeImageLoc;
string SceneImageLoc;
string pathToGazeText;
string pathToGaze;

// Queue data structures: Brendan
Queue_Eye   LeftEyeQueue;
Queue_Eye   RightEyeQueue;
Queue_Scene SceneQueue;

// Mutex objects, for locking of image queue: Brendan
pthread_mutex_t mutexEyeL  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexEyeR  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexScene = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexFlag = PTHREAD_MUTEX_INITIALIZER;

// Int to work as data collection flag. 1 if data is being collected. Set to 0 when done.
int studyFlag;

/* **************************************************************************************** */
/* *************************************** FUNCTIONS  ************************************* */
/* **************************************************************************************** */

int main (int argc, char ** argv) {
	
	string prName;
	string taskNum;

	cout << "Enter subject name: "; 
	getline(cin, prName);  
	cout << "Enter task number: ";
	getline(cin, taskNum);  

	RightEyeImageLoc = "C:\\Users\\Rakshit\\Documents\\ETGData" + prName + "\\" + taskNum + "\\RightEyeImages";
	LeftEyeImageLoc = "C:\\Users\\Rakshit\\Documents\\ETGData\\" + prName + "\\" + taskNum + "\\LeftEyeImages";
	SceneImageLoc = "C:\\Users\\Rakshit\\Documents\\ETGData\\" + prName + "\\" + taskNum + "\\SceneImages";
	pathToGaze = "C:\\Users\\Rakshit\\Documents\\ETGData\\" + prName + "\\" + taskNum + "\\GazeData";
	pathToGazeText = pathToGaze	+ "\\Gaze_Data.txt";

	if (_mkdir(RightEyeImageLoc.c_str()) == 0)
	{
		_mkdir(LeftEyeImageLoc.c_str());
		_mkdir(SceneImageLoc.c_str());
		_mkdir(pathToGaze.c_str());
		printf("Directories successfully created");
	}
	else 
	{
		printf("Directories already existed. Deleting.");
		_rmdir(RightEyeImageLoc.c_str());
		_rmdir(LeftEyeImageLoc.c_str());
		_rmdir(SceneImageLoc.c_str());
		_rmdir(pathToGaze.c_str());

		_mkdir(RightEyeImageLoc.c_str());
		_mkdir(LeftEyeImageLoc.c_str());
		_mkdir(SceneImageLoc.c_str());
		_mkdir(pathToGaze.c_str());

		printf("Directories successfully created");
	}

	cout << RightEyeImageLoc << "\n";
	cout << LeftEyeImageLoc << "\n";
	cout << SceneImageLoc << "\n";
	cout << pathToGazeText << "\n";



	// Compression parameters	
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

	iViewRC rc;

	memset (&gServer, 0, sizeof(iViewHost));

	if ( (rc = ParseCommandLine (argc, argv)) || (rc = Setup ()) || (rc = Subscribe ()) || (rc = Start ()) || (rc = SetupCalib ()))
		return RC_OPERATION_FAILED;

	printf ("Receiving data...\n");

	// Create file for writing Gaze Data
	GazeFile.open(pathToGazeText);
	GazeFile << "EyeFrameNumber,SceneFrameNumber,serverTime,BporX,BporY,Year,Month,Day,Hour,Minute,Second,Millisecond\n";

	// Create queue's for scene and eye images
	LeftEyeQueue    = createEyeQueue();
	RightEyeQueue   = createEyeQueue();
	SceneQueue      = createSceneQueue();
	studyFlag       = 1;

	WaitForUserInteraction ();

	// destroy the queue's
	LeftEyeQueue.destroy_Eye(&LeftEyeQueue);
	RightEyeQueue.destroy_Eye(&RightEyeQueue);
	SceneQueue.destroy_Scene(&SceneQueue);

	printf ("Cleaning up...\n");

	Cleanup ();
	GazeFile.close();

	return 0;
}

/* **************************************************************************************** */

/**
 * Create and initiate a scene queue
 */
Queue_Scene createSceneQueue () {
    Queue_Scene queue;
    queue.size          = 0;
    queue.head          = NULL;
    queue.tail          = NULL;
    queue.push_Scene    = &push_Scene;
    queue.pop_Scene     = &pop_Scene;
	queue.destroy_Scene = &destroy_SceneQueue;
    return queue;
}

/**
 * Create and initiate an eye queue
 */
Queue_Eye createEyeQueue () {
    Queue_Eye queue;
    queue.size        = 0;
    queue.head        = NULL;
    queue.tail        = NULL;
    queue.push_Eye    = &push_Eye;
    queue.pop_Eye     = &pop_Eye;
	queue.destroy_Eye = &destroy_EyeQueue;
	return queue;
}

/**
 * Push an item into scene queue, if this is the first item,
 * both queue->head and queue->tail will point to it,
 * otherwise the oldtail->next and tail will point to it.
 */
void push_Scene (Queue_Scene* queue, iViewDataStreamSceneImage* img) {
    // Create a new node
    Node_Scene* n = (Node_Scene*) malloc (sizeof(Node_Scene));
    n->img        = img;
    n->next       = NULL;

    if (queue->head == NULL) { // no head
        queue->head = n;
    } else{
        queue->tail->next = n;
    }
    queue->tail = n;
    queue->size++;
}
/**
 * Return and remove the first item from scene queue
 */
iViewDataStreamSceneImage* pop_Scene (Queue_Scene* queue) {
    // get the first item
    Node_Scene* head = queue->head;
    iViewDataStreamSceneImage* img = head->img;
    // move head pointer to next node, decrease size
    queue->head = head->next;
    queue->size--;
    // free the memory of original head
    free(head);
    return img;
}

/**
 * Push an item into scene queue, if this is the first item,
 * both queue->head and queue->tail will point to it,
 * otherwise the oldtail->next and tail will point to it.
 */
void push_Eye (Queue_Eye* queue, iViewDataStreamEyeImage* img) {
    // Create a new node
    Node_Eye* n = (Node_Eye*) malloc (sizeof(Node_Eye));
    n->img        = img;
    n->next       = NULL;

    if (queue->head == NULL) { // no head
        queue->head = n;
    } else{
        queue->tail->next = n;
    }
    queue->tail = n;
    queue->size++;
}
/**
 * Return and remove the first item from scene queue
 */
iViewDataStreamEyeImage* pop_Eye (Queue_Eye* queue) {
    // get the first item
    Node_Eye* head = queue->head;
    iViewDataStreamEyeImage* img = head->img;
    // move head pointer to next node, decrease size
    queue->head = head->next;
    queue->size--;
    // free the memory of original head
    free(head);
    return img;
}

void destroy_EyeQueue(Queue_Eye* queue) {
	iViewDataStreamEyeImage* temp;
	
	//destroy by popping until empty
	while (queue->size > 0) {
		temp = queue->pop_Eye(queue);
		free(temp);
	}
}

void destroy_SceneQueue(Queue_Scene* queue) {
	iViewDataStreamSceneImage* temp;

	//destroy by popping until empty
	while (queue->size > 0) {
		temp = queue->pop_Scene(queue);
		free(temp);
	}
}

static void * _cdecl worker_EyeThreadL(void * param) {
	iViewDataStreamEyeImage* imgToWrite;
	// first grab lock, check size. If has element then pop it and save off
	pthread_mutex_lock(&mutexEyeL);
	if (LeftEyeQueue.size > 0) {
		imgToWrite = LeftEyeQueue.pop_Eye(&LeftEyeQueue);
		
	}
	pthread_mutex_unlock(&mutexEyeL);
	writeImage(imgToWrite->imageData, imgToWrite->eyeFrameNumber, LeftEyeImageLoc, compression_params);

	//optionally could add small time (ms) sleep here for worker thread
	//Might have to have another variable that signifies that the study is over, then can call pthread_exit() here

	pthread_mutex_lock(&mutexFlag);
	if (mutexFlag == 0) {
		pthread_exit(0);
		return NULL;
	}
	pthread_mutex_unlock(&mutexFlag);

	
}

static void * _cdecl worker_EyeThreadR(void * param) {
	iViewDataStreamEyeImage* imgToWrite;
	// first grab lock, check size. If has element then pop it and save off
	pthread_mutex_lock(&mutexEyeR);
	if (RightEyeQueue.size > 0) {
		imgToWrite = RightEyeQueue.pop_Eye(&RightEyeQueue);

	}
	pthread_mutex_unlock(&mutexEyeR);
	writeImage(imgToWrite->imageData, imgToWrite->eyeFrameNumber, RightEyeImageLoc, compression_params);

	//optionally could add small time (ms) sleep here for worker thread

	pthread_mutex_lock(&mutexFlag);
	if (mutexFlag == 0) {
		pthread_exit(0);
		return NULL;
	}
	pthread_mutex_unlock(&mutexFlag);
}

static void * _cdecl worker_SceneThread(void * param) {
	iViewDataStreamSceneImage* imgToWrite;
	// first grab lock, check size. If has element then pop it and save off
	pthread_mutex_lock(&mutexScene);
	if (LeftEyeQueue.size > 0) {
		imgToWrite = SceneQueue.pop_Scene(&SceneQueue);

	}
	pthread_mutex_unlock(&mutexScene);
	writeImage(imgToWrite->imageData, imgToWrite->sceneFrameNumber, SceneImageLoc, compression_params);

	//optionally could add small time (ms) sleep here for worker thread

	pthread_mutex_lock(&mutexFlag);
	if (mutexFlag == 0) {
		pthread_exit(0);
		return NULL;
	}
	pthread_mutex_unlock(&mutexFlag);
}

/* **************************************************************************************** */

/**
 * This function will be called from MyCallback() when a new gaze sample is available.
 */
void handleGazeSample (iViewDataStreamGazeSample const * const gazeSample) {
	


	// round the x and y coordinate
	gGazeX = (int) (gazeSample->pointOfRegard.x + 0.5);
	gGazeY = (int) (gazeSample->pointOfRegard.y + 0.5);

	// Print to console only if requested.
	if (gShowGaze)
		/*
		printf ("Gaze Sample %u: serverTime=%ums, por=%d/%d, synchDate %04d.%02d.%02d %02d:%02d:%02d.%03d\n", gazeSample->eyeFrameNumber,
		        (uint32_t) (gazeSample->timestamp / 1000000), gGazeX, gGazeY,
				gazeSample->year, gazeSample->month, gazeSample->day,
				gazeSample->hour, gazeSample->minute, gazeSample->second,
				gazeSample->millisecond
				);
				*/
		/*
		sprintf (str2Write, "EyeFrameNumber %u, SceneFrameNumber %u, serverTime %ums, BporX %d, BporY %d, synchDate %04d.%02d.%02d %02d:%02d:%02d.%03d\n", 
				gazeSample->eyeFrameNumber,
				gazeSample->sceneFrameNumber,
		        (uint32_t) (gazeSample->timestamp / 1000000), gGazeX, gGazeY,
				gazeSample->year, gazeSample->month, gazeSample->day,
				gazeSample->hour, gazeSample->minute, gazeSample->second,
				gazeSample->millisecond
				);
				*/

		sprintf(str2Write,"%u,%u,%u,%d,%d,%04d,%02d,%02d,%02d,%02d,%02d,%03d\n",
				gazeSample->eyeFrameNumber, gazeSample->sceneFrameNumber,
		        (uint32_t) (gazeSample->timestamp / 1000000), gGazeX, gGazeY,
				gazeSample->year, gazeSample->month, gazeSample->day,
				gazeSample->hour, gazeSample->minute, gazeSample->second,
				gazeSample->millisecond);
		GazeFile << str2Write;


	return;
}

/* **************************************************************************************** */

/**
 * This function will be called from MyCallback() when a new eye image is available.
 */
void handleEyeImage (iViewDataStreamEyeImage * image) {

	switch (image->eye) {

		case EYE_LEFT:

			//displayLeftEyeImage (image->imageData);
			//writeImage(image->imageData, image->eyeFrameNumber, LeftEyeImageLoc, compression_params);
			
			//bjohn: instead of writing image we add to queue using mutex lock.
			pthread_mutex_unlock(&mutexEyeL);
			LeftEyeQueue.push_Eye(&LeftEyeQueue, image);
			pthread_mutex_lock(&mutexEyeL);
			
			break;

		case EYE_RIGHT:

			//displayRightEyeImage (image->imageData);
			//writeImage(image->imageData, image->eyeFrameNumber, RightEyeImageLoc, compression_params);
			
			//bjohn: instead of writing image we add to queue using mutex lock.
			pthread_mutex_unlock(&mutexEyeR);
			RightEyeQueue.push_Eye(&RightEyeQueue, image);
			pthread_mutex_lock(&mutexEyeR);
			
			break;

		case EYE_UNKNOWN:
			fprintf (stderr, "ERROR in callback: unknown eye type.\n");
			break;
		}
		
	return;
}

/* **************************************************************************************** */

/**
 * This function will be called from MyCallback() when a new scene image is available.
 */
void handleSceneImageWithGaze (iViewDataStreamSceneImage * image) {
	//gCurrentFrameNumber = image->sceneFrameNumber;
	//displaySceneImage (image->imageData);
	//writeImage(image->imageData, image->sceneFrameNumber, SceneImageLoc, compression_params);

	//bjohn: instead of writing image we add to queue using mutex lock.
	pthread_mutex_unlock(&mutexScene);
	SceneQueue.push_Scene(&SceneQueue, image);
	pthread_mutex_lock(&mutexScene);

	return;
}

/* **************************************************************************************** */

/**
 * This function will be called from MyCallback() when a new scene image is available.
 */
void handleH264DecodedSceneImage (iViewDataStreamSceneImage * image) {

	//gCurrentFrameNumber = image->sceneFrameNumber;

	/*
	// Draw gaze onto scene only if requested.
	if (gShowSceneImagesWithGaze || gShowSceneH264ImagesWithGaze) {
		drawGazeOverlay (image->imageData, gGazeX, gGazeY);
	}
	*/
	//displaySceneImage (image->imageData);
	//writeImage(image->imageData, image->sceneFrameNumber, SceneImageLoc, compression_params);

	//bjohn: instead of writing image we add to queue using mutex lock.
	pthread_mutex_unlock(&mutexScene);
	SceneQueue.push_Scene(&SceneQueue, image);
	pthread_mutex_lock(&mutexScene);
	return;
}

/* **************************************************************************************** */

/**
 * The callback analyses the result type and delegates operation to dedicated event handler.
 */
void MyCallback (iViewTicket * const ticket) {
	//cout<<"Func call "<<ticket->functionName<<"\n";
	// this should never happen
	if (NULL == ticket)
		return;

	// extract the result
	iViewResult const * const result = ticket->result;

	// Check if we got a 1pt calibration response
	if (gTicketCalibration1Pt == ticket) {
		printf ("Sending 1pt calibration data %s successful.\n", RC_NO_ERROR == ticket->returnCode ? "was" : "was not");
		iView_ReleaseTicket( &gTicketCalibration1Pt );
		return;
	}

	// Check if we got a 3pt calibration response
	for (int i=0; i<3; i++) {
		if (gTicketCalibration3Pt[i] == ticket) {
			printf ("Sending 3pt calibration data %s successful.\n", RC_NO_ERROR == ticket->returnCode ? "was" : "was not");
			iView_ReleaseTicket( &gTicketCalibration3Pt[i] );
			return;
		}
	}

	// if the ticket carries no result or if it's not a data stream ticket, we're not interested
	if (NULL == result || IVIEWRESULT_SUBSCRIBE_DATASTREAM != result->type)
		return;

	// cast to the proper result type
	iViewDataStream const * const stream = (iViewDataStream const *) result->data;

	if(stream->lastStreamEntity == IVIEWDATASTREAM_END){
		fprintf(stderr,"Got last stream Entity\n");
		return;
	}
	//cout << "Stream type" << stream->type << "\n";
	switch (stream->type) {

		case IVIEWDATASTREAM_GAZE_INFORMATION:

			handleGazeSample ((iViewDataStreamGazeSample const *) stream->data);
			break;

		case IVIEWDATASTREAM_EYEIMAGES_LEFT:

			handleEyeImage ((iViewDataStreamEyeImage*) stream->data);
			break;

		case IVIEWDATASTREAM_EYEIMAGES_RIGHT:

			handleEyeImage ((iViewDataStreamEyeImage*) stream->data);
			break;

		case IVIEWDATASTREAM_SCENEIMAGES:
		case IVIEWDATASTREAM_SCENEIMAGES_WITH_GAZE:
		case IVIEWDATASTREAM_SCENEIMAGES_H264_DECODED_WITH_GAZE:

			handleSceneImageWithGaze ((iViewDataStreamSceneImage*) stream->data);
			break;

		case IVIEWDATASTREAM_SCENEIMAGES_H264_DECODED:

			handleH264DecodedSceneImage ((iViewDataStreamSceneImage*) stream->data);
			break;

	}

	iView_ReleaseResult (ticket);
	return;
}

/* **************************************************************************************** */

void Usage (char const * const cmd) {

	fprintf (stderr, "\nUSAGE\n\t%s [OPTION]...\n\n", cmd ? cmd : "");
	fprintf(stderr, "OPTIONS\n\n");
	fprintf(stderr, "\t--help               to print this information\n");
	fprintf(stderr, "\t--show-gaze          to print the gaze coordinates to the console\n");
	fprintf(stderr, "\t--show-eyes          to open two windows displaying the eye images\n");
	fprintf(stderr, "\t--scale-eyes f       rescale eye images by multiplying image size by f\n");
	fprintf(stderr, "\t--show-scene         to open a window displaying the scene images (the\n");
	fprintf(stderr, "\t                     scene video is decoded by the server and full image\n");
	fprintf(stderr, "\t                     will be transferred to the remote viewer)\n");
	fprintf(stderr, "\t--show-scene-h264    to open a window displaying the scene images (the\n");
	fprintf(stderr, "\t                     scene video is transferred as H.264 stream,\n");
	fprintf(stderr, "\t                     decoded by the remote viewer)\n");
	fprintf(stderr, "\t--show-scene-with-gaze	to open a window displaying the scene images (the\n");
	fprintf(stderr, "\t                     scene video is decoded by the server and already\n");
	fprintf(stderr, "\t                     contains the gaze cursor, full image is transferred to\n");
	fprintf(stderr, "\t                     the remote viewer)\n");
	fprintf(stderr, "\t--show-scene-h264-with-gaze    to open a window displaying the scene images\n");
	fprintf(stderr, "\t                     (the scene video is transferred as H.264 stream,\n");
	fprintf(stderr, "\t                     decoded by the client and contains a gaze cursor\n");
	fprintf(stderr, "\t                     overlay created by the client)\n");
	fprintf(stderr, "\t--scale-scene f      rescale scene images by multiplying image size by f\n");
	fprintf(stderr, "\t--server ip_address  connect to the specified remote iViewNG server,\n");
	fprintf(stderr, "\t                     the server can be setup (device parameters) by this\n");
	fprintf(stderr, "\t                     Tutorial-RemoteViewer\n");
	fprintf(stderr, "\t--iviewetg ip_address  connect to the specified remote iViewNG server\n");
	fprintf(stderr, "\t                     which had already been setup by an iViewETG-Client.\n");
	fprintf(stderr, "\t                     This Tutorial-RemoteViewer will not submit any new device\n");
	fprintf(stderr, "\t                     parameters, it will only try to subscribe and listen to\n");
	fprintf(stderr, "\t                     the data it wants. Whether the data can be received\n");
	fprintf(stderr, "\t                     depends on the parameters that iViewETG-Client has set\n");
	fprintf(stderr, "\t--calibrate1pt       to perform a 1-point calibration\n");
	fprintf(stderr, "\t--calibrate3pt       to perform a 3-point calibration\n");
	fprintf(stderr, "\t--samplingrate n     set eye tracking sampling rate to 30 or 60Hz or 120hz\n");
	fprintf(stderr, "\t--scene30            scene 30hz\n");
	fprintf(stderr, "\t--scene24            scene 24hz\n");
	fprintf(stderr, "\t--gazeoverlay        turns on gaze overlay mode. if you have wireless\n");
	fprintf(stderr, "\t                     observation license, this is the only mode you can use.\n");
	fprintf(stderr, "\t--shutdownserver     specifies that the server is to be shutdown by an\n");
	fprintf(stderr, "\t                     explicit command; if not specified, server's behavior\n");
	fprintf(stderr, "\t                     defines how it will behave when this client leaves\n");
	fprintf(stderr, "\t--timemaster         specifies that this sdk instance is the time master of\n");
	fprintf(stderr, "\t                     the server it connects to. If not set, the server will\n");
	fprintf(stderr, "\t                     use its local date and time to stamp the gaze samples.\n");
	fprintf(stderr, "\t                     User must ensure there is only one time master running\n");
	fprintf(stderr, "\t                     at a time (otherwise the times will compete).\n");
	fprintf(stderr, "\t--ica                to turn on the ICA recording mode.\n");
	fprintf(stderr, "\t                     The ICA Recording Mode summarizes custom hardware\n");
	fprintf(stderr, "\t                     settings to especially enable optimal results when\n");
	fprintf(stderr, "\t                     the SMI ETG are used in conjunction with the Index\n");
	fprintf(stderr, "\t                     of Cognitive Activity from Eye Tracking Inc.\n");
	fprintf(stderr, "\t                     This mode might lead to a limited robustness\n");
	fprintf(stderr, "\t                     against external IR light. Please make sure that\n");
	fprintf(stderr, "\t                     you use this mode only in combination with the ICA.\n");
	fprintf(stderr, "\t                     This mode does not run with 120 Hz.\n");
	fprintf(stderr, "\t--ica-off            turns off ICA recording mode.\n");
	fprintf(stderr, "\t                     If you have enabled it before, you can turn it off.\n");
	exit (-1);
}

/* **************************************************************************************** */

iViewRC ParseCommandLine (int argc, char ** argv) {

	// if not directed to server, use local server
	gServer.connectionType = IVIEW_SERVERADRRESS_SHAREDMEMORY;

	// check each parameter
	for (int i = 1; i < argc; i++) {
		//cout<<"<<<- arg "<<argv[i]<<"\n";
		if (argv [i] [0] == '-') {

			if (0 == strncmp (argv [i], "--help", MAX (strlen (argv [i]), strlen ("--help")))) {
				Usage (argv [0]);
			}

			if (0 == strncmp (argv [i], "--server", MAX (strlen (argv [i]), strlen ("--server")))) {

				if (argc <= ++i) {
					fprintf (stderr, "ERROR: missing argument for parameter '--server'");
					Usage(argv[0]);
				}

				gServer.connectionType = IVIEW_SERVERADRRESS_IPV4;
				unsigned int argLen = strlen(argv [i]);
				strncpy (gServer.hostAddress.ipAddress.ipv4, argv [i], MIN (argLen, HOSTADDRESSLENGTH_IPV4 - 1));
				gServer.hostAddress.port = 0;

				printf ("Server host: '%s:%u'\n", gServer.hostAddress.ipAddress.ipv4, gServer.hostAddress.port);
				continue;
			}

			if (0 == strncmp (argv [i], "--iviewetg", MAX (strlen (argv [i]), strlen ("--iviewetg")))) {

				if (argc <= ++i) {
					fprintf (stderr, "ERROR: missing argument for parameter '--iviewetg'");
					Usage(argv[0]);
				}

				gServer.connectionType = IVIEW_SERVERADRRESS_IPV4;
				unsigned int argLen = strlen(argv [i]);
				strncpy (gServer.hostAddress.ipAddress.ipv4, argv [i],  MIN (argLen, HOSTADDRESSLENGTH_IPV4 - 1));
				gServer.hostAddress.port = 0;

				gRemoteIsIviewEtg = 1;

				printf ("Server (with iViewETG) host: '%s:%u'\n", gServer.hostAddress.ipAddress.ipv4, gServer.hostAddress.port);
				continue;
			}

			if (0 == strncmp (argv [i], "--samplingrate", MAX (strlen (argv [i]), strlen ("--samplingrate")))) {
				if (argc <= ++i) {
					fprintf (stderr, "ERROR: missing argument for parameter '--samplingrate'");
					Usage(argv[0]);
				}

				switch (atoi(argv [i])) {
				case 30:
					gSamplingRate = IVIEWSAMPLERATE_ETG_30;
					break;
				case 60:
					gSamplingRate = IVIEWSAMPLERATE_ETG_60;
					break;
				case 120:
					gSamplingRate = IVIEWSAMPLERATE_ETG_120;
					break;
				default:
					fprintf (stderr, "Invalid sampling rate: neither '30' nor '60' nor '120'.\n");
					Usage(argv[0]);
				}

				continue;
			}

			if (0 == strncmp (argv [i], "--show-gaze", MAX (strlen (argv [i]), strlen ("--show-gaze")))) {
				gShowGaze = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--show-eyes", MAX (strlen (argv [i]), strlen ("--show-eyes")))) {
				gShowEyeImages = 1;
				continue;
			}
			//cout<<" parameter "<<argv [i]<<"\n";
			if (0 == strncmp (argv [i], "--show-scene", MAX (strlen (argv [i]), strlen ("--show-scene")))) {
				gShowSceneImages = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--show-scene-h264", MAX (strlen (argv [i]), strlen ("--show-scene-h264")))) {
				gShowSceneH264 = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--show-scene-with-gaze", MAX (strlen (argv [i]), strlen ("--show-scene-with-gaze")))) {
				gShowSceneImagesWithGaze = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--show-scene-h264-with-gaze", MAX (strlen (argv [i]), strlen ("--show-scene-h264-with-gaze")))) {
				gShowSceneH264ImagesWithGaze = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--calibrate1pt", MAX (strlen (argv [i]), strlen ("--calibrate1pt")))) {
				gCalibrate1Pt = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--calibrate3pt", MAX (strlen (argv [i]), strlen ("--calibrate3pt")))) {
				gCalibrate3Pt = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--scale-eyes", MAX (strlen (argv [i]), strlen ("--scale-eyes")))) {

				if (argc <= ++i) {
					fprintf (stderr, "ERROR: missing argument for parameter '--scale-eyes'");
					Usage(argv[0]);
				}

				gScaleEyes = atof(argv[i]);
				continue;
			}

			if (0 == strncmp (argv [i], "--scale-scene", MAX (strlen (argv [i]), strlen ("--scale-scene")))) {

				if (argc <= ++i) {
					fprintf (stderr, "ERROR: missing argument for parameter '--scale-scene'");
					Usage(argv[0]);
				}

				gScaleScene = atof(argv[i]);
				gScaleSceneSet = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--scene24", MAX (strlen (argv [i]), strlen ("--scene24")))) {
				gScene24 = 1;
				continue;
			}
			if (0 == strncmp (argv [i], "--scene30", MAX (strlen (argv [i]), strlen ("--scene30")))) {
				gScene30 = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--gazeoverlay", MAX (strlen (argv [i]), strlen ("--gazeoverlay")))) {
				gGazeOverlay = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--shutdownserver", MAX (strlen (argv [i]), strlen ("--shutdownserver")))) {
				gShutdownServer = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--timemaster", MAX (strlen (argv [i]), strlen ("--timemaster")))) {
				gTimeMaster = 1;
				continue;
			}

			if (0 == strncmp (argv [i], "--ica", MAX (strlen (argv [i]), strlen ("--ica")))) {

				if (IVIEWSAMPLERATE_ETG_120 == gSamplingRate) {
					fprintf (stderr, "ICA cannot run with 120Hz sampling.\n");
					Usage(argv[0]);
				}

				gEyeExposureMode = IVIEWEYECAMEXPOSUREMODE_ICA;
				continue;
			}

			if (0 == strncmp (argv [i], "--ica-off", MAX (strlen (argv [i]), strlen ("--ica-off")))) {
				gEyeExposureMode = IVIEWEYECAMEXPOSUREMODE_NORMAL;
				continue;
			}

			fprintf (stderr, "ERROR: unknown parameter: %s", argv[i]);
			Usage (argv [0]);
		}
	}

	if (!gShowGaze && !gShowEyeImages && !gShowSceneImages && !gShowSceneH264 && !gShowSceneImagesWithGaze && ! gShowSceneH264ImagesWithGaze) {
		fprintf (stderr, "ERROR: Refusing to run without showing any data stream!\n");
		Usage(argv[0]);
	}

	if (gShowSceneImages && gShowSceneH264 && gShowSceneImagesWithGaze && gShowSceneH264ImagesWithGaze) {
		fprintf (stderr, "ERROR: Please specify either --show-scene or --show-scene-h264 or --show-scene-with-gaze.\n");
		Usage(argv[0]);
	}

	if ((gCalibrate1Pt || gCalibrate3Pt)
			&& !gShowSceneH264 && !gShowGaze && !gShowSceneImagesWithGaze && ! gShowSceneH264ImagesWithGaze) {
		fprintf (stderr, "ERROR: To calibrate, you must specify either --show-scene-with-gaze or --show-scene-h264-with-gaze.\n");
		Usage(argv[0]);
	}

	gServer.device = IVIEWDEVICE_ETG_CAMERAPLAYBACK;

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

iViewRC Setup () {

	fprintf (stderr, "Starting...");

	CALL_API(iView_Init (IVIEWSDK_IVNG));

	// Start server only if shared memory is used
	if (IVIEW_SERVERADRRESS_SHAREDMEMORY == gServer.connectionType)
	{
		printf ("\n Shared memory IS used.");
		CALL_API(iView_StartServer (NULL, L"--mode etg"));
	}

	fprintf (stderr, "done...\n");

	CALL_API(iView_SetCallback (MyCallback));

	fprintf (stderr, "Connecting...");

	CALL_API(iView_CreateTicket (&gTicketConnect));
	CALL_API(iView_Connect (gTicketConnect, &gServer, TICKET_WAIT_MS, 4, NULL ));

	fprintf(stderr, "done.\n");

	/* ---------------------------------------------------------------------------- */
	// an example on how to add an own license key
	//
	// this would be the code to send a license key (provided you have a custom license key).
	// but normally, the license is contained in the ETG itself, so you do not need to add
	// a special license key
	//
	fprintf(stderr, "Sending license...");

	CALL_API(iView_CreateTicket (&gTicketAddLicense));
	CALL_API(iView_StartCatchingTickets());
	CALL_API(iView_AddLicenseKey (gTicketAddLicense, ""));

	if (0 == iView_WaitForTicket (gTicketAddLicense, TICKET_WAIT_MS)) {
		printf ("Failed to add license.\n");
		return RC_OPERATION_FAILED;
	}

	fprintf(stderr, "done.\n");


	/* ---------------------------------------------------------------------------- */
	// set time master
	if (gTimeMaster) {
		fprintf(stderr, "Telling Server that this SDK instance is the time master..");
		CALL_API( iView_SetProcessAsTimeMaster() );
		fprintf(stderr, "done\n");
	}

	/* ---------------------------------------------------------------------------- */
	// submit device parameters to server

	if (gRemoteIsIviewEtg) {
		fprintf(stderr, "Remote iViewNG-Server is running with iViewETG-Client, will not submit device parameters\n");
		fprintf(stderr, "done.\n");
		return RC_NO_ERROR;
	}

	fprintf(stderr, "Submitting device parameters...");

	iViewDeviceParametersEtgCameraPlayback specificParameters;

	specificParameters.outputDirectory = L".";
	specificParameters.baseFilename = L"Tutorial-RemoteViewer";
	specificParameters.operationMode = IVIEWTRACKINGMODE_CURRENT;

	specificParameters.cameraResolutionScene = IVIEWRESOLUTION_CURRENT;
	specificParameters.samplingRateSceneCam = IVIEWSAMPLINGRATE_CURRENT;
	if (gScene24) {
		specificParameters.cameraResolutionScene = IVIEWRESOLUTION_ETG_1280x960;
		specificParameters.samplingRateSceneCam = IVIEWSAMPLERATE_ETG_24;
	}
	if (gScene30) {
		specificParameters.cameraResolutionScene = IVIEWRESOLUTION_ETG_960x720;
		specificParameters.samplingRateSceneCam = IVIEWSAMPLERATE_ETG_30;
	}

	specificParameters.whiteBalanceProgram = IVIEWWHITEBALANCE_ETG_AUTO;
	specificParameters.audioState = IVIEWTRACKINGMODE_CURRENT;
	specificParameters.overlaySpecList = NULL;

	if (gSamplingRate == IVIEWSAMPLERATE_ETG_120) {
		// set to 120Hz eye tracking
		specificParameters.samplingRateEyeCam = IVIEWSAMPLERATE_ETG_120;
		specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_320x240;
		//specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_1280x960;
	}
	else if (gSamplingRate == IVIEWSAMPLERATE_ETG_60) {
		// set to 60Hz eye tracking
		specificParameters.samplingRateEyeCam = IVIEWSAMPLERATE_ETG_60;
		specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_320x240;
	}
	else {
		// set to 30Hz eye tracking
		specificParameters.samplingRateEyeCam = IVIEWSAMPLERATE_ETG_30;
		specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_320x240;
	}


	if (gGazeOverlay) {
		fprintf(stderr, "Gaze Overlay mode specified, will set eye frequency to 30Hz, scene frequency to 24Hz\n");

		specificParameters.gazeOverlayMode = IVIEWGAZEOVERLAYMODE_ACTIVATED;

		specificParameters.samplingRateEyeCam = IVIEWSAMPLERATE_ETG_30;
		specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_320x240;

		specificParameters.cameraResolutionScene = IVIEWRESOLUTION_ETG_1280x960;
		specificParameters.samplingRateSceneCam = IVIEWSAMPLERATE_ETG_24;
	}

	specificParameters.eyeCamExposureMode = gEyeExposureMode;


	// because the license might take a while we sleep a bit here
	iView_Sleep(5000);

	iViewDeviceParameters deviceParameters;
	deviceParameters.deviceType = IVIEWDEVICE_ETG_CAMERAPLAYBACK;
	deviceParameters.parameters = &specificParameters;

	CALL_API(iView_CreateTicket (&gTicketDeviceParameters));
	CALL_API(iView_StartCatchingTickets());
	CALL_API(iView_SetDeviceParameters (gTicketDeviceParameters, &deviceParameters));


	if (0 == iView_WaitForTicket (gTicketDeviceParameters, TICKET_WAIT_MS)) {
		printf ("Failed to set device parameters.\n");
		return RC_OPERATION_FAILED;
	}

	// Check if device parameters could be set
	if (gTicketDeviceParameters->returnCode != RC_NO_ERROR) {
		const size_t MAX_LEN = 1024;
		wchar_t ticketErrorStr[MAX_LEN];

		size_t len = MAX_LEN;
		iView_iViewRcToString( ticketErrorStr, &len, gTicketDeviceParameters->returnCode );

		wprintf( L"Failed to set device parameters, ticket return code '%s'\n",
			ticketErrorStr );


		if (gTicketDeviceParameters->returnCode == RC_INVALID_LICENSE) {
			fprintf(stderr, "got invalid license, will try with gazeoverlay mode\n");
			fprintf(stderr, "Gaze Overlay mode specified, will set eye frequency to 30Hz, scene frequency to 24Hz\n");

			specificParameters.gazeOverlayMode = IVIEWGAZEOVERLAYMODE_ACTIVATED;

			specificParameters.samplingRateEyeCam = IVIEWSAMPLERATE_ETG_30;
			specificParameters.cameraResolutionEye = IVIEWRESOLUTION_ETG_960x720;

			specificParameters.cameraResolutionScene = IVIEWRESOLUTION_ETG_1280x960;
			specificParameters.samplingRateSceneCam = IVIEWSAMPLERATE_ETG_24;


			iView_ReleaseTicket( &gTicketDeviceParameters );
			CALL_API(iView_CreateTicket (&gTicketDeviceParameters));
			CALL_API(iView_SetDeviceParameters (gTicketDeviceParameters, &deviceParameters));

			// wait some time for directgaze switch
			iView_Sleep(TICKET_WAIT_MS);

			// Check if device parameters could be set
			if (gTicketDeviceParameters->returnCode != RC_NO_ERROR) {
				const size_t MAX_LEN = 1024;
				wchar_t ticketErrorStr[MAX_LEN];

				size_t len = MAX_LEN;
				iView_iViewRcToString( ticketErrorStr, &len, gTicketDeviceParameters->returnCode );

				wprintf( L"Failed to set device parameters, ticket return code '%s'\n",
					ticketErrorStr );

				return gTicketDeviceParameters->returnCode;
			} else {
				fprintf(stderr, "done.\n");

				return RC_NO_ERROR;
			}

		}


		return gTicketDeviceParameters->returnCode;
	}

	fprintf(stderr, "done.\n");

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

/**
 * Make sure SetupCalib() is called after Start() as calibration needs enough accumulated
 * scene images.
 */
iViewRC SetupCalib () {

	if (!gCalibrate1Pt && !gCalibrate3Pt)
		return RC_NO_ERROR;

	fprintf(stderr, "Submitting calibration parameters...");

	// We remember the time when we call setup calib, so that we do not propagate
	// calibration points too soon.
	iViewTicket * timeTicket = NULL;
	CALL_API(iView_CreateTicket(&timeTicket));
	CALL_API(iView_GetLocalTime(timeTicket));
	gTimeOfSetupCalibMsec = ((iViewTime *) timeTicket->result->data)->time / 1000000;
	CALL_API(iView_ReleaseTicket(&timeTicket));

	// Sleep a bit to ensure that enough scene frames have been
	// accumulated by the server (necessary for calibration)
	// before sending the calibration setup.
	iView_Sleep(gSetupCalibCooldownMsec);

	// send calibration parameters
	iViewCalibrationParametersEtg specificParametersSetup;
	specificParametersSetup.calibrationDistance = 0;

	iViewCalibrationParameters parametersSetup;

	parametersSetup.type = IVIEWCALIBRATIONTYPE_ETG_1POINTSPHERICAL;
	if (gCalibrate3Pt) {
		parametersSetup.type = IVIEWCALIBRATIONTYPE_ETG_3POINT;
	}

	parametersSetup.applyAtFrame = -1;
	parametersSetup.parameters = &specificParametersSetup;

	iViewTicket * ticket = NULL;
	CALL_API(iView_CreateTicket(&ticket));
	CALL_API(iView_StartCatchingTickets());
	CALL_API(iView_SetupCalibration(ticket, &parametersSetup));

	if (0 == iView_WaitForTicket (ticket, TICKET_WAIT_MS)) {
		printf ("Failed to set calibration parameters.\n");
		//return -1;
	}

	// and start calibration
	CALL_API(iView_ReleaseTicket(&ticket));
	CALL_API(iView_CreateTicket(&ticket));
	CALL_API(iView_StartCatchingTickets());
	CALL_API(iView_StartCalibration(ticket));

	if (0 == iView_WaitForTicket (ticket, TICKET_WAIT_MS)) {
		printf ("Failed to start calibration.\n");
		//return -1;
	}

	fprintf(stderr, "done.\n");

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

iViewRC Subscribe () {

	fprintf (stderr, "Subscribing data streams...");

	/* -------------------------------------------- */
	if (gShowGaze || gShowSceneH264ImagesWithGaze) {
		//cout<<" <- gShowGaze \n";
		fprintf (stderr, "gaze information...");

		iViewStreamSubscription subscriptionGaze;
		memset (&subscriptionGaze, 0, sizeof(iViewStreamSubscription));
		subscriptionGaze.streamType = IVIEWDATASTREAM_GAZE_INFORMATION;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionGaze));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionGaze, &subscriptionGaze));
		
		if (0 == iView_WaitForTicket (gTicketSubscriptionGaze, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe gaze data stream.\n");
			//return -1;
		}
	}

	/* -------------------------------------------- */
	if (gShowEyeImages) {
		//cout<<" <- gShowEyeImages \n";

		fprintf (stderr, "eye images...");

		iViewDataStreamSpecSampleRate sampleRateEyes;
		sampleRateEyes.type = IVIEWDATASTREAMSPEC_SAMPLE_RATE;
		sampleRateEyes.next = NULL;
		sampleRateEyes.sampleRate = gSamplerateEyes;

		iViewStreamSubscription subscriptionLeftEye;
		subscriptionLeftEye.streamType = IVIEWDATASTREAM_EYEIMAGES_LEFT;
		subscriptionLeftEye.streamSpec = (iViewDataStreamSpec*) &sampleRateEyes;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionLeftEye));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionLeftEye, &subscriptionLeftEye));

		if (0 == iView_WaitForTicket (gTicketSubscriptionLeftEye, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe left eye stream.\n");
			//return -1;
		}

		iViewStreamSubscription subscriptionRightEye;
		subscriptionRightEye.streamType = IVIEWDATASTREAM_EYEIMAGES_RIGHT;
		subscriptionRightEye.streamSpec = (iViewDataStreamSpec*) &sampleRateEyes;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionRightEye));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionRightEye, &subscriptionRightEye));

		if (0 == iView_WaitForTicket (gTicketSubscriptionRightEye, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe right eye stream.\n");
			return -1;
		}
	}

	/* -------------------------------------------- */
	if (gShowSceneImages) {

		fprintf (stderr, "scene image (decoded)...");

		iViewDataStreamSpecSampleRate sampleRateScene;
		sampleRateScene.type = IVIEWDATASTREAMSPEC_SAMPLE_RATE;
		sampleRateScene.next = NULL;
		sampleRateScene.sampleRate = gSamplerateScene;

		iViewStreamSubscription subscriptionScene;
		subscriptionScene.streamType = IVIEWDATASTREAM_SCENEIMAGES;
		subscriptionScene.streamSpec = (iViewDataStreamSpec*) &sampleRateScene;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionScene));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionScene, &subscriptionScene));

		if (0 == iView_WaitForTicket (gTicketSubscriptionScene, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe scene stream.\n");
			return -1;
		}
	}

	/* -------------------------------------------- */
	if (gShowSceneImagesWithGaze) {
		fprintf (stderr, "scene image (decoded) with gaze...");

		iViewDataStreamSpecSampleRate sampleRateScene;
		sampleRateScene.type = IVIEWDATASTREAMSPEC_SAMPLE_RATE;
		sampleRateScene.next = NULL;
		sampleRateScene.sampleRate = gSamplerateScene;

		iViewStreamSubscription subscriptionScene;
		subscriptionScene.streamType = IVIEWDATASTREAM_SCENEIMAGES_WITH_GAZE;
		subscriptionScene.streamSpec = (iViewDataStreamSpec*) &sampleRateScene;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionSceneWithGaze));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionSceneWithGaze, &subscriptionScene));

		if (0 == iView_WaitForTicket (gTicketSubscriptionSceneWithGaze, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe scene stream.\n");
			return -1;
		}
	}

	/* -------------------------------------------- */
	if (gShowSceneH264 || gShowSceneH264ImagesWithGaze) {

		fprintf (stderr, "scene image (H.264)...");

		// For transfer through low quality networks, we better use TCP communication since a single lost
		// packet leaves the video stream unusable until the next I-frame is transmitted successfully.
		iViewDataStreamSpecQualityOfService communicationQuality;
		communicationQuality.type = IVIEWDATASTREAMSPEC_QUALITY_OF_SERVICE;
		communicationQuality.next = NULL;
		communicationQuality.quality = IVIEWQOS_LOSSLESS;

		iViewStreamSubscription subscriptionScene;
		subscriptionScene.streamType = IVIEWDATASTREAM_SCENEIMAGES_H264_DECODED;
		subscriptionScene.streamSpec = (iViewDataStreamSpec*) &communicationQuality;

		CALL_API(iView_CreateTicket(&gTicketSubscriptionSceneH264));
		CALL_API(iView_StartCatchingTickets());
		CALL_API(iView_SubscribeDataStream(gTicketSubscriptionSceneH264, &subscriptionScene));

		if (0 == iView_WaitForTicket (gTicketSubscriptionSceneH264, TICKET_WAIT_MS)) {
			printf ("Failed to set Subscribe scene stream.\n");
			return RC_OPERATION_FAILED;
		}
	}

	fprintf(stderr, "done.\n");

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

iViewRC Start () {

	fprintf (stderr, "Starting data acquisition...");

	CALL_API(iView_CreateTicket(&gTicketStartAcquisition));
	CALL_API(iView_StartCatchingTickets());
	CALL_API(iView_StartDataAcquisition(gTicketStartAcquisition));

	if (0 == iView_WaitForTicket (gTicketStartAcquisition, TICKET_WAIT_MS)) {
		printf ("Failed to Start data acquisition.\n");
		return RC_OPERATION_FAILED;
	}


	fprintf(stderr, "done.\n");

	if (gCalibrate1Pt) {
		printf("Please click once on the currently fixated point to calibrate.\n");
		gCalibrationPointsToDo = 1;
	}
	if (gCalibrate3Pt) {
		printf("Please click once on the currently fixated point to calibrate 1st point of a 3pt calibration.\n");
		gCalibrationPointsToDo = 3;
	}

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

iViewRC Calibrate (const unsigned int coordinateX, const unsigned int coordinateY) {

	if (gCalibrationPointsToDo == 0)
		return RC_INVALID_STATE;

	// Setup, sending calibration type
	if (gCalibrate1Pt) {
		printf ("Sending data for 1-point-calibration (%u/%u) at frame %u.\n", coordinateX, coordinateY,
				gCurrentFrameNumber);
	} else if (gCalibrate3Pt) {
		printf ("Sending data point %u/3 for 3-point-calibration (%u/%u) at frame %u.\n",
				3-gCalibrationPointsToDo+1, coordinateX, coordinateY, gCurrentFrameNumber);
	} else {
		return RC_INVALID_STATE;
	}

	iViewCalibrationData1Point calibrationPoint;
	calibrationPoint.coordinateX = coordinateX;
	calibrationPoint.coordinateY = coordinateY;

	iViewCalibrationData data;
	data.type = IVIEWCALIBRATIONDATATYPE_1POINT;
	data.sceneFrameNumber = gCurrentFrameNumber;
	data.unit = IVIEWCALIBRATIONUNIT_PIXEL;
	data.usage = 0;
	data.id = 0;
	data.parameters = &calibrationPoint;

	if (gCalibrate1Pt) {
		CALL_API(iView_CreateTicket (&gTicketCalibration1Pt));
		CALL_API(iView_SetCalibrationData (gTicketCalibration1Pt, &data));
		// We don't want to block here, thus we receive the reply ticket via MyCallback().
	}

	if (gCalibrate3Pt) {
		CALL_API(iView_CreateTicket (&gTicketCalibration3Pt[3-gCalibrationPointsToDo]));
		CALL_API(iView_SetCalibrationData (gTicketCalibration3Pt[3-gCalibrationPointsToDo], &data));
		// We don't want to block here, thus we receive the reply ticket via MyCallback().
	}


	gCalibrationPointsToDo--;

	return RC_NO_ERROR;
}

/* **************************************************************************************** */

iViewRC Cleanup () {

	fprintf (stderr, "Unsubscribing data stream...");

	CALL_API(iView_CreateTicket(&gTicketUnsubscription));
	CALL_API(iView_UnsubscribeDataStream(gTicketUnsubscription, IVIEWDATASTREAM_GAZE_INFORMATION));

	fprintf(stderr, "done.\n");

	/* ---------------------------------------------------------------------------- */
	/* stop data acquisition */

	// We will stop acquisition if the server is not being controlled by an iViewETG-Client
	if (gRemoteIsIviewEtg == 0) {
		fprintf(stderr, "Stopping data acquisition...");

		CALL_API(iView_CreateTicket(&gTicketStopAcquisition));
		CALL_API(iView_StopDataAcquisition(gTicketStopAcquisition));

		fprintf(stderr, "done.\n");

		iView_Sleep (1000);
	}

	/* ---------------------------------------------------------------------------- */
	/* shut down */
	fprintf(stderr, "Shutting down API...");

	if (gTicketConnect) CALL_API(iView_ReleaseTicket (&gTicketConnect));
	if (gTicketAddLicense) CALL_API(iView_ReleaseTicket (&gTicketAddLicense));
	if (gTicketDeviceParameters) CALL_API(iView_ReleaseTicket (&gTicketDeviceParameters));
	if (gTicketStartAcquisition) CALL_API(iView_ReleaseTicket (&gTicketStartAcquisition));
	if (gTicketSubscriptionGaze) CALL_API(iView_ReleaseTicket (&gTicketSubscriptionGaze));
	if (gTicketUnsubscription) CALL_API(iView_ReleaseTicket (&gTicketUnsubscription));
	if (gTicketStopAcquisition) CALL_API(iView_ReleaseTicket (&gTicketStopAcquisition));

	// We will terminate the server only if explicit flag has been set.
	if (gShutdownServer) {
		// terminate the iViewNG server and give it some time to shut down
		fprintf(stderr, "\nShuttdown down Server...\n");
		CALL_API(iView_ShutdownServer (30000));
	}

	// shut down the API
	iView_Sleep(1000);

	//	disconnect from server
	CALL_API(iView_Disconnect ());

	CALL_API(iView_Shutdown ());

	fprintf(stderr, "done.\n");

	return RC_NO_ERROR;

}

#include <conio.h> // for kbhit()

/**
 * Wait for user interaction as the data stream is being received via the callback function.
 */
void WaitForUserInteraction () {

	//start threads for each queue
	pthread_t threadL, threadR, threadScene;
	
	pthread_create(&threadL, NULL, worker_EyeThreadL, NULL);
	pthread_create(&threadR, NULL, worker_EyeThreadR, NULL);
	pthread_create(&threadScene, NULL, worker_SceneThread, NULL);


	// Wait for data to arrive via callback
	printf ("Press 'ESC' to exit.\n");

	int key = -1;

	// while key is not 'Escape'
	while (key != 0x1B && key != 'q' && key != 'x' && key != 'Q' && key != 'X') {

		if (!kbhit ()) {
			// no key pressed, just wait.
			iView_Sleep (10);
			continue;
		}
		// key pressed!
		key = getch();
	}

	//end threads for each queue, done by setting flag
	//TODO: test this works.
	pthread_mutex_lock(&mutexFlag);
	studyFlag = 0;
	pthread_mutex_unlock(&mutexFlag);

	return;
}