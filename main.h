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

//struct for generic Queue, will be used for storing pointers to images (L_eye, R_eye, Scene)
typedef struct Node
{
  void *data;
  struct Node *next;
}node;

typedef struct QueueList
{
    int sizeOfQueue;
    size_t memSize;
    node *head;
    node *tail;
}Queue;

void queueInit(Queue *q, size_t memSize);
int enqueue(Queue *, const void *);
void dequeue(Queue *, void *);
void queuePeek(Queue *, void *);
void clearQueue(Queue *);
int getQueueSize(Queue *);

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

