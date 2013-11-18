// =======================================================================================
// Sixense Controller Server: Extremely simple program to make the inputs from a Sixense
//                    Motion controller available to Maya.
//
// Author: Evan Harper (harper4@gmail.com)
//

// =======================================================================================

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include <maya/mocapserver.h>
#include <maya/mocapserial.h>
#include <maya/mocaptcp.h>

#include <sixense.h>
#include <sixense_math.hpp>
#include <sixense_utils/mouse_pointer.hpp>
#include <sixense_utils/derivatives.hpp>
#include <sixense_utils/button_states.hpp>
#include <sixense_utils/event_triggers.hpp>
#include <sixense_utils/controller_manager/controller_manager.hpp>

#include <winsock2.h>
#pragma comment( lib, "wsock32.lib" )     // This will link to wsock32.lib, could set up for proj settings?

template<class T> __forceinline T square(const T& val)  { return val*val; } // not needed?

#define ServerName "SixenseMayaServer"

static int handle_client(int client_fd);
std::string controller_manager_text_string;


CapChannel LeftTrigger;
CapChannel RightTrigger;


// This is the callback that gets registered with the sixenseUtils::controller_manager. It will get called each time the user completes
// one of the setup steps so that the game can update the instructions to the user. If the engine supports texture mapping, the 
// controller_manager can prove a pathname to a image file that contains the instructions in graphic form.
// The controller_manager serves the following functions:
//  1) Makes sure the appropriate number of controllers are connected to the system. The number of required controllers is designaged by the
//     game type (ie two player two controller game requires 4 controllers, one player one controller game requires one)
//  2) Makes the player designate which controllers are held in which hand.
//  3) Enables hemisphere tracking by calling the Sixense API call sixenseAutoEnableHemisphereTracking. After this is completed full 360 degree
//     tracking is possible.
void controller_manager_setup_callback( sixenseUtils::controller_manager::setup_step step ) {
	printf ("%s \n", "Setting Up Controllers");
	if( sixenseUtils::getTheControllerManager()->isMenuVisible() ) {

		// Turn on the flag that tells the graphics system to draw the instruction screen instead of the controller information. The game
		// should be paused at this time.
		//controller_manager_screen_visible = true;

		// Ask the controller manager what the next instruction string should be.
		controller_manager_text_string = sixenseUtils::getTheControllerManager()->getStepString();
		printf ("%s \n", controller_manager_text_string);
		// We could also load the supplied controllermanager textures using the filename: sixenseUtils::getTheControllerManager()->getTextureFileName();

	} else {

		// We're done with the setup, so hide the instruction screen.
		//controller_manager_screen_visible = false;
		printf ("%s \n", "Controller Setup Complete");

	}

}

int main(int argc, char const* argv[])
{
	sixenseInit();
	int curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
	printf ("curStep :: %d \n", curStep);
	
	sixenseUtils::getTheControllerManager()->registerSetupCallback( controller_manager_setup_callback );
	sixenseUtils::getTheControllerManager()->setGameType( sixenseUtils::controller_manager::ONE_PLAYER_TWO_CONTROLLER );
	printf ("curStep :: %d \n", curStep);
	printf ("%s \n", "Sixense Capture Server For Maya2011, version 0.1");
	printf ("%s \n", "Author: Evan Harper, August 2011");
    // Create the channels for the xenon controller.  Everything is created as a 1-d
    // "unknown" channel.  This kinda sucks, since we know that there are boolean values,
    // but there doesn't appear to be support for these types in the mocap server API.
    // Just map them to 0/1 floats.
#define CREATE_UNK_CHANNEL(Name) Name = CapCreateChannel( #Name, CAP_USAGE_UNKNOWN, 1)
    {

        CREATE_UNK_CHANNEL(LeftTrigger);
        CREATE_UNK_CHANNEL(RightTrigger);

    }
	sixenseSetActiveBase(0);
	printf ("Base (0) :: %d \n", sixenseIsBaseConnected(0));
	curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
	printf ("curStep :: %d \n", curStep);
#undef CREATE_UNK_CHANNEL
    for(;;)
    {
        /*
         * Set up the server socket and wait for a connection.
         */
		printf ("%s \n", "Server Created, Waiting for Client Connection");
		curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
		printf ("curStep :: %d \n", curStep);
        int client_fd = CapServe(ServerName);
		
        if (client_fd < 0)
        {
            CapError(-1, CAP_SEV_FATAL, ServerName "0", NULL);
            exit(1);
        }
		printf ("%s \n", "Maya Connection Established, Initializing Sixense Device");
		sixenseUtils::getTheControllerManager()->setGameType( sixenseUtils::controller_manager::ONE_PLAYER_TWO_CONTROLLER );
		curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
		float value = 1.0f; 
		CapSetData(LeftTrigger, &value);
		printf ("curStep :: %d \n", curStep);
		printf ("Base (0) :: %d \n", sixenseIsBaseConnected(0));
		// Init the controller manager. This makes sure the controllers are present, assigned to left and right hands, and that
		// the hemisphere calibration is complete.
		
        /* Handle client requests */
		printf ("%s \n", "Calling Handle Clients");
		printf ("curStep :: %d \n", curStep);
        int status = handle_client(client_fd);
		printf ("%s \n", "Other Side of Handle Clients");
        if (status < 0)
        {
            CapError(-1, CAP_SEV_FATAL, ServerName "1", NULL);
        }

        /* Shutdown the client */
        closesocket(client_fd);
		sixenseExit();	



	

	
    }

    return 0;
}



void check_for_button_presses( sixenseAllControllerData *acd ) {
	//printf ("%s \n", "Checking For Button Presses");
	// Ask the controller manager which controller is in the left hand and which is in the right
	int left_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::controller_manager::P1L );
	int right_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::controller_manager::P1R );



	// First use the 'ButtonStates' class to flash the object when the 1 button is pressed, or the trigger is pulled.
	// ButtonStates is a simple class that reports when a button's state just transitioned from released to pressed
	// or vice versa. It also detects when the trigger crosses a programmable threshold.
	static sixenseUtils::ButtonStates left_states, right_states;

	left_states.update( &acd->controllers[left_index] );
	right_states.update( &acd->controllers[right_index] );


	// Or if the trigger was pulled
	if( left_states.triggerJustPressed() ) {
		float value = 1.0f; 
		CapSetData(LeftTrigger, &value);
		printf ("%s \n", "Left Trigger");
	}

	if( right_states.triggerJustPressed() ) {
		float value = 0.0f; 
		CapSetData(LeftTrigger, &value);
		printf ("%s \n", "Right Trigger");
	}
}

static void get_data(int client_fd)
{
	printf ("%s \n", "Getting Data");
	sixenseSetActiveBase(0);
	sixenseAllControllerData acd;
	sixenseGetAllNewestData( &acd );
	sixenseUtils::getTheControllerManager()->update( &acd );

	check_for_button_presses( &acd );

}

static int handle_client(int client_fd)
{

    for (;;)
    {
        fd_set rd_fds;
        FD_ZERO(&rd_fds);
        FD_SET((unsigned int)client_fd, &rd_fds);

        int status = select(FD_SETSIZE, &rd_fds, NULL, NULL, NULL);
        if (status < 0)
        {
            /* Otherwise, give a fatal error message */
            CapError(client_fd, CAP_SEV_FATAL, ServerName, "select failed");
            CapError(client_fd, CAP_SEV_FATAL, "select", NULL);
            exit(1);
        }
        else if (status == 0)
        {
            /* We got a timeout?  Try again */
            continue;
        }
        else
        {
            /* There is data on the client file descriptor */
            CapCommand cmd = CapGetCommand(client_fd);
            switch (cmd)
            {
                case CAP_CMD_QUIT:
                    return 0;

                case CAP_CMD_ERROR:
                    return -1;

                case CAP_CMD_AUTHORIZE:
                    status = CapAuthorize(client_fd, 1);
                    break;

                case CAP_CMD_INIT:  /* Initial client/server handshake */
                    status = CapInitialize(client_fd, ServerName);
                    break;

                case CAP_CMD_VERSION:   /* Send version information */
                    status = CapVersion(client_fd, ServerName, "0.1", "Sixense Controller capture server for maya- v0.1");
                    break;

                case CAP_CMD_INFO:
                    status = CapInfo(client_fd, 0.0, 0.0, 0.0, 0, 1);
                    break;

                case CAP_CMD_DATA:  /* Send frame data */
					printf ("%s \n", "Calling Get Data From Handle_Client");
                    get_data(client_fd);
                    status = CapData(client_fd);
                    break;

                case CAP_CMD_START_RECORD:
                case CAP_CMD_STOP_RECORD:
                default:
                    status = CapError(client_fd, CAP_SEV_ERROR, ServerName,
                                      "Unknown or unsupported server command.");
                    break;
            }

            if (status < 0)
            {
                return -1;
            }
        }
    }
}
