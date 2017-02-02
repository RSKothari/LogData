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

#include <highgui.h>
#include <cv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iViewNG-DataStreams.h>

using namespace std;
using namespace cv;

// Open a window to display the scene images
void createSceneImageWindow ();

// Open a window to display the left eye images
void createLeftEyeImageWindow ();

// Open a window to display the right eye images
void createRightEyeImageWindow ();

// Display the scene image if it is valid
void displaySceneImage (iViewImage *image);

// Draw a circle at the specified position into the provided image.
void drawGazeOverlay(iViewImage *image, const int x, const int y);

// Display the left eye image if it is valid
void displayLeftEyeImage (iViewImage *image);

// Rakshit: Write the left image at given location
void writeImage (iViewImage *image, int, string, vector<int>);

// Display the right eye image if it is valid
void displayRightEyeImage (iViewImage *image);

// Run the event loop that draws the image data and receives window events
// The delay defines the time available for these tasks (0=until key pressed)
void runImageDisplayEventLoop (int delay);

// Close the display window
void closeSceneImageDisplay ();

// Close the left eye window
void closeLeftEyeImageDisplay ();

// Close the right eye window
void closeRightEyeImageDisplay ();

// This function handles mouse events on the scene image display.
// It is only active if the program waits for the user to input
// a calibration point.
void onMouseInSceneImage (int event, int x, int y, int flags, void* param);

// Sets the mouse callback that the event loops calls to handle mouse events
void setMouseCallbackOnSceneImageDisplay ();

// Removes the mouse callback from the event loop
void removeMouseCallbackOnSceneImageDisplay ();