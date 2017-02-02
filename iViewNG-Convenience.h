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
#include <iViewNG-DataStreams.h>

extern iViewRC _iView_Convenience_rc;
extern wchar_t _iView_Convenience_errorString[128];

#define CALL_API(_fun) { \
		if ( RC_NO_ERROR != (_iView_Convenience_rc = _fun) ) { \
			size_t _iView_Convenience_errorStringSize = sizeof(_iView_Convenience_errorString); \
			iView_iViewRcToString (_iView_Convenience_errorString, &_iView_Convenience_errorStringSize, _iView_Convenience_rc); \
			fprintf (stderr, "ERROR in '" #_fun "': %ls\r\n", _iView_Convenience_errorString); \
			return _iView_Convenience_rc; \
		}\
}

extern iViewRC iView_StartCatchingTickets ();
extern int iView_WaitForTicket (iViewTicket const * const, const unsigned int);
extern int iView_WaitForDataStream (const iViewDataStreamType, const unsigned int, const unsigned int);

