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

#include "iViewNG-Convenience.h"
#include <iViewNG-Utility.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

/* **************************************************************************************** */

static const unsigned int IVIEW_TICKETS_MAX = 1024;

iViewRC _iView_Convenience_rc;
wchar_t _iView_Convenience_errorString[128];

unsigned int * _iView__receivedTickets = NULL;
unsigned int * _iView__receivedStreamCounter = NULL;

int _iView__waitingTime = 0;

IVIEW_CALLBACK _iView_UserCallback = NULL;

/* **************************************************************************************** */
/* **************************************************************************************** */

/**
 * A callback function which is used as API callback. If you register this callback to the iViewNG API,
 * all iViewNG API calls will run into this function. Use this only if you want your program to block until
 * a ticket is returned / a data stream event is received.
 */
void _iView__callback (iViewTicket * const ticket) {

	// return if there is no ticket
	if (NULL == ticket) {
		//_iView__unlockCallback ();
		return;
	}

	//wprintf (L"_iView__callback() : receiving ticket #%u (function '%s') with returnCode %u and result 0x%x\n",
	//ticket->ticketNumber, ticket->functionName ? ticket->functionName : L"(unset)", ticket->returnCode,
	//ticket->result);

	// memorize that we received this ticket number
	if (IVIEW_TICKETS_MAX <= ticket->ticketNumber) {
		wprintf (L"ERROR: Too many tickets, please increase IVIEW_TICKETS_MAX!\n");
		return;
	}
	_iView__receivedTickets [ticket->ticketNumber]++;

	iViewResult const * const result = ticket->result;

	// return if there is no result
	if (NULL == result) {
		//_iView__unlockCallback ();
		return;
	}

	// return if the result is no data stream
	if (IVIEWRESULT_SUBSCRIBE_DATASTREAM != result->type) {
		//_iView__unlockCallback ();
		return;
	}

	iViewDataStream const * const stream = (iViewDataStream const *) result->data;

	// break if the stream type is higher than expected
	if (NUMBER_OF_IVIEWDATASTREAMS <= stream->type) {
		//_iView__unlockCallback ();
		return;
	}

	// memorize that we received this ticket number
	_iView__receivedStreamCounter [stream->type]++;

	//_iView__unlockCallback ();
	return;
}

/* **************************************************************************************** */
/* **************************************************************************************** */

/**
 *
 */
iViewRC iView_StartCatchingTickets () {

	// if required, release array of fetched ticket numbers
	if (_iView__receivedTickets) {
		memset (_iView__receivedTickets, 0, IVIEW_TICKETS_MAX * sizeof(unsigned int));
		free (_iView__receivedTickets);
		_iView__receivedTickets = NULL;
	}
	// reallocate
	_iView__receivedTickets = (unsigned int *) calloc (IVIEW_TICKETS_MAX, sizeof(unsigned int));
	if (NULL == _iView__receivedTickets)
		return RC_OUT_OF_MEMORY;

	// if required, release array of fetched stream events
	if (_iView__receivedStreamCounter) {
		memset (_iView__receivedStreamCounter, 0, NUMBER_OF_IVIEWDATASTREAMS * sizeof(unsigned int));
		free (_iView__receivedStreamCounter);
		_iView__receivedStreamCounter = NULL;
	}
	// reallocate
	_iView__receivedStreamCounter = (unsigned int *) calloc (NUMBER_OF_IVIEWDATASTREAMS, sizeof(unsigned int));
	if (NULL == _iView__receivedStreamCounter)
		return RC_OUT_OF_MEMORY;

	// save user callback
	_iView_UserCallback = iView_GetCallback ();

	// and apply convenience callback
	return iView_SetCallback (_iView__callback);
}

/* **************************************************************************************** */
/* **************************************************************************************** */

/**
 * Block the caller until either
 *
 * - the ticket number was called
 * - the function blocked for waitMilliseconds.
 *
 * The function unlocks the mutex when starting to wait to enable the API to enter the callback. The mutex
 * is NOT locked afterwards. If you want to prevent API module from entering the callback, use
 * ApiTestCallback::lockCallback() explicitly!
 *
 * Don't forget to call reset() to wipe out older results!
 */
int iView_WaitForTicket (iViewTicket const * const ticket, const unsigned int waitMilliseconds) {

	const unsigned int WAIT_PER_ITERATION = 50;
	unsigned int waitingTime = 0;

	//_iView__unlockCallback ();

	do {
		if (_iView__receivedTickets [ticket->ticketNumber]) {
			break;
		}

		// if not found, sleep a bit
		iView_Sleep (WAIT_PER_ITERATION);
		waitingTime += WAIT_PER_ITERATION;
	}
	while (waitingTime <= waitMilliseconds);

	// restore user callback
	iView_SetCallback (_iView_UserCallback);

	//_iView__lockCallback ();

	return _iView__receivedTickets [ticket->ticketNumber] != 0;
}

/* **************************************************************************************** */
/* **************************************************************************************** */

/**
 * Block the caller until either
 *
 * - numberOfEvents events of the type IVIEWRESULT_SUBSCRIBE_DATASTREAM with the provided stream type were received or
 * - the function blocked for waitMilliseconds.
 *
 * The function unlocks the mutex when starting to wait to enable the API to enter the callback. The mutex
 * is NOT locked afterwards. If you want to prevent API module from entering the callback, use
 * ApiTestCallback::lockCallback() explicitly!
 *
 * Don't forget to call reset() to wipe out older results!
 */
int iView_WaitForDataStream (const iViewDataStreamType streamType, const unsigned int waitMilliseconds,
        const unsigned int numberOfEvents) {

	const unsigned int WAIT_PER_ITERATION = 50;

	unsigned int waitingTime = 0;

//	_iView__unlockCallback ();

	while (numberOfEvents > _iView__receivedStreamCounter [streamType] && waitingTime <= waitMilliseconds) {

		iView_Sleep (WAIT_PER_ITERATION);
		waitingTime += WAIT_PER_ITERATION;
	}

	// restore user callback
	iView_SetCallback (_iView_UserCallback);

	//_iView__lockCallback ();

	return numberOfEvents <= _iView__receivedStreamCounter [streamType];
}

/* **************************************************************************************** */
/* **************************************************************************************** */