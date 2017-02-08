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
// Code modified by Brendan John (bmj8778@rit.edu) & Rakshit Kothari (rsk3900@rit.edu)
// to include queue data structures and to handle data recieved wirelessly through router.
// 
// Credits: C Queue implementation credited to Bai Ben (http://ben-bai.blogspot.com/2012/04/simple-queue-data-structure-in-ansi-c.html) 
//
// -----------------------------------------------------------------------
#pragma once

#include <iViewNG-Core.h>
#include <vector>

/* **************************************************************************************** */
/* ************************************* DATATYPES **************************************** */
/* **************************************************************************************** */

// Scale factor applied to the scene image for downscaling.
extern float gScaleEyes;
extern float gScaleScene;
extern char gScaleSceneSet;
extern char gCalibrationPointsToDo;
extern unsigned int gTimeOfSetupCalibMsec;
extern unsigned int gSetupCalibCooldownMsec;

/**
 * The Node struct for use with eye images,
 * contains pointer to image data and then points to the next node.
 */
typedef struct Node_Eye {
    iViewDataStreamEyeImage* img;
    struct Node_Eye* next;
} Node_Eye;
/**
 * The Queue_Eye struct, contains the pointers that
 * point to first node and last node, and the size of the Queue.
 */
typedef struct Queue_Eye {
    Node_Eye* head;
    Node_Eye* tail;
	int size;
    void (*push_Eye) (struct Queue_Eye*, iViewDataStreamEyeImage*); // add item to tail
    // get item from head and remove it from queue
    iViewDataStreamEyeImage* (*pop_Eye) (struct Queue_Eye*);
	void(*destroy_Eye) (struct Queue_Eye*);
} Queue_Eye;

/**
 * The Node struct for use with scene images,
 * contains pointer to image data and then points to the next node.
 */
typedef struct Node_Scene {
    iViewDataStreamSceneImage* img;
    struct Node_Scene* next;
} Node_Scene;
/**
 * The Queue_Scene struct, contains the pointers that
 * point to first node and last node, and the size of the Queue.
 */
typedef struct Queue_Scene {
    Node_Scene* head;
    Node_Scene* tail;
	int size;
    void (*push_Scene) (struct Queue_Scene*, iViewDataStreamSceneImage*); // add item to tail
    // get item from head and remove it from queue
    iViewDataStreamSceneImage* (*pop_Scene) (struct Queue_Scene*);
	void(*destroy_Scene) (struct Queue_Scene*);
} Queue_Scene;

/**
 * Push an item into eye queue, if this is the first item,
 * both queue->head and queue->tail will point to it,
 * otherwise the oldtail->next and tail will point to it.
 */
void push_Eye (Queue_Eye* queue, iViewDataStreamEyeImage* img);
/**
 * Return and remove the first item.
 */
iViewDataStreamEyeImage* pop_Eye (Queue_Eye* queue);

/**
 * Push an item into scene queue, if this is the first item,
 * both queue->head and queue->tail will point to it,
 * otherwise the oldtail->next and tail will point to it.
 */
void push_Scene (Queue_Scene* queue, iViewDataStreamSceneImage* img);
/**
 * Return and remove the first item.
 */
iViewDataStreamSceneImage* pop_Scene (Queue_Scene* queue);

/**
* Create and initiate a Queue_Eye
*/
Queue_Eye createEyeQueue();

/**
 * Create and initiate a Queue_Scene
 */
Queue_Scene createSceneQueue ();

/**
* Function that frees memory used for eye queue
*/
void destroy_EyeQueue(Queue_Eye* queue);

/**
* Function that frees memory used for eye queue
*/
void destroy_SceneQueue(Queue_Scene* queue);

/**
* Worker functions that will be assigned to each thread that saves off images from
* the corresponding queue.
*/
void * worker_EyeThreadL(void * param);
void * worker_EyeThreadR(void * param);
void * worker_SceneThread(void * param);

/* **************************************************************************************** */
/* *************************************** FUNCTIONS  ************************************* */
/* **************************************************************************************** */
extern iViewRC Calibrate (const unsigned int, const unsigned int);

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

/* **************************************************************************************** */
/* *************************************** FUNCTIONS  ************************************* */
/* **************************************************************************************** */

