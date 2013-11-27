//-
// ==========================================================================
// Copyright 2011 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

// Description:
// This example demonstrates how to use a secondary thread to generate
// translate data which controls an object.
//

/*
 // MEL:
 loadPlugin hydraDevice;
 string $node = `createNode hydraDevice`;
 string $cube[] = `camera`;
 setAttr ($cube[0]+".rotateOrder") 2; // set to zxy rotation order (minimizes gimbal euler angles)
 connectAttr ( $node + ".outputTranslate" ) ( $cube[0] + ".translate" );
 connectAttr ( $node + ".outputRotate" ) ( $cube[0] + ".rotate" );
 // ADD VECTOR DATA
 string $loc[] = `spaceLocator -p 0 0 0`;
 


	setAttr ( $node + ".live" ) 1;
 
 $r = `createNode record`;
 connectAttr camera1.tx ($r+".input");
 $r = `createNode record`;
 connectAttr camera1.ty ($r+".input");
 $r = `createNode record`;
 connectAttr camera1.tz ($r+".input");
 $r = `createNode record`;
 connectAttr camera1.rx ($r+".input");
 $r = `createNode record`;
 connectAttr camera1.ry ($r+".input");
 $r = `createNode record`;
 connectAttr camera1.rz ($r+".input");
 
 play -rec;
*/

#include <stdlib.h>

#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h>
#include <maya/MEulerRotation.h>

#include <maya/MQuaternion.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MTransformationMatrix.h>

#include "api_macros.h"
#include <maya/MIOStream.h>

#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MPxThreadedDeviceNode.h>



// ==========================================================================
// Hydra Device Libraries
#include <sixense.h>
#include <sixense_math.hpp>
#ifdef WIN32
#include <sixense_utils/mouse_pointer.hpp>
#endif
#include <sixense_utils/derivatives.hpp>
#include <sixense_utils/button_states.hpp>
#include <sixense_utils/event_triggers.hpp>
#include <sixense_utils/controller_manager/controller_manager.hpp>

// ==========================================================================
 
// flags that the controller manager system can set to tell the graphics system to draw the instructions
// for the player
//static bool controller_manager_screen_visible = true;
//std::string controller_manager_text_string;


class hydraDeviceNode : public MPxThreadedDeviceNode
{

public:
						hydraDeviceNode();
	virtual 			~hydraDeviceNode();
	
	virtual void		postConstructor();
	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );
	virtual void		threadHandler();
	virtual void		threadShutdownHandler();

	static void*		creator();
	static MStatus		initialize();

public:

    static MObject		output;
	static MObject		outputTranslate;
	static MObject 		outputTranslateX;
	static MObject		outputTranslateY;
	static MObject 		outputTranslateZ;
	static MObject		outputRotate;
	static MObject 		outputRotateX;
	static MObject		outputRotateY;
	static MObject 		outputRotateZ;
	static MObject 		outputTrigger;
	static MObject 		outputButtons;
    static MObject 		outputJoystick;
	static MObject 		outputJoystickX;
	static MObject 		outputJoystickY;
	static MObject 		outputJoystickZ;
	static MObject		offsetTranslate;
	static MObject 		offsetTranslateX;
	static MObject		offsetTranslateY;
	static MObject 		offsetTranslateZ;
    static MObject      offsetRotate;
    static MObject      offsetRotateX;
    static MObject      offsetRotateY;
    static MObject      offsetRotateZ;
    static MObject      zoomX;
    static MObject      zoomY;
    static MObject      zoomSpeedX;
    static MObject      zoomSpeedY;

	static MTypeId		id;

private:
};

MTypeId hydraDeviceNode::id( 0x00081051 );
MObject hydraDeviceNode::output;
MObject hydraDeviceNode::outputTranslate;
MObject hydraDeviceNode::outputTranslateX;
MObject hydraDeviceNode::outputTranslateY;
MObject hydraDeviceNode::outputTranslateZ;
MObject hydraDeviceNode::outputRotate;
MObject hydraDeviceNode::outputRotateX;
MObject hydraDeviceNode::outputRotateY;
MObject hydraDeviceNode::outputRotateZ;
MObject hydraDeviceNode::outputTrigger;
MObject hydraDeviceNode::outputButtons;
MObject hydraDeviceNode::outputJoystick;
MObject hydraDeviceNode::outputJoystickX;
MObject hydraDeviceNode::outputJoystickY;
MObject hydraDeviceNode::outputJoystickZ;
MObject hydraDeviceNode::offsetTranslate;
MObject hydraDeviceNode::offsetTranslateX;
MObject hydraDeviceNode::offsetTranslateY;
MObject hydraDeviceNode::offsetTranslateZ;
MObject hydraDeviceNode::offsetRotate;
MObject hydraDeviceNode::offsetRotateX;
MObject hydraDeviceNode::offsetRotateY;
MObject hydraDeviceNode::offsetRotateZ;

MObject hydraDeviceNode::zoomX;
MObject hydraDeviceNode::zoomY;
MObject hydraDeviceNode::zoomSpeedX;
MObject hydraDeviceNode::zoomSpeedY;


hydraDeviceNode::hydraDeviceNode() 
{

}

hydraDeviceNode::~hydraDeviceNode()
{
	destroyMemoryPools();
}

void hydraDeviceNode::postConstructor()
{
	MObjectArray attrArray;
	attrArray.append( hydraDeviceNode::outputTranslate);
	setRefreshOutputAttributes( attrArray );

	// we'll be reading one set of translate x,y, z's at a time
	createMemoryPools( 24, 10, sizeof(double));
    
    //createMemoryPools (12, 3, sizeof(Ptr)) ;
}

static double getRandomX()
{
	// rand() is not thread safe for getting
	// the same results in different threads.
	// But this does not matter for this simple
	// example.
    
    
	const double kScale = 10.0;
	double i = (double) rand();
	return ( i / RAND_MAX ) * kScale;
}


void hydraDeviceNode::threadHandler()
{
	MStatus status;
	setDone( false );
    
	sixenseInit();
	int curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
	printf ("curStep :: %d \n", curStep);
	
	//sixenseUtils::getTheControllerManager()->registerSetupCallback( controller_manager_setup_callback );
	//sixenseUtils::getTheControllerManager()->setGameType( sixenseUtils::controller_manager::ONE_PLAYER_TWO_CONTROLLER );
	printf ("curStep :: %d \n", curStep);
	printf ("%s \n", "Hydra Input Device for Maya, version 0.1");
	printf ("%s \n", "Author: Evan Harper (harper4@gmail.com), Adam Burke (adam@adamburke.net), Nov 2013");

    
    
	while ( ! isDone() )
	{
		// Skip processing if we
		// are not live
		if ( ! isLive() )
			continue;

		MCharBuffer buffer;
		status = acquireDataStorage(buffer);
		if ( ! status )
			continue;

		beginThreadLoop();
		{
            
            const double pi = 3.14159265358979323846; // CHANGE THIS TO LIBRARY CONST
            const double world_scale = 0.02;
            
            //printf ("%s \n", "Getting Data");
            sixenseSetActiveBase(0);
            sixenseAllControllerData acd;
            sixenseGetAllNewestData( &acd );
            
            sixenseUtils::getTheControllerManager()->update( &acd );
            
            static sixenseUtils::ButtonStates left_states, right_states;
            left_states.update( &acd.controllers[0] );
            right_states.update( &acd.controllers[1] );
            
			double p_x = (double)acd.controllers[0].pos[0]; 
			double p_y = (double)acd.controllers[0].pos[1]; 
			double p_z = (double)acd.controllers[0].pos[2];   
                        
            
            double q_x = (double)acd.controllers[0].rot_quat[0];
            double q_y = (double)acd.controllers[0].rot_quat[1];
            double q_z = (double)acd.controllers[0].rot_quat[2];
            double q_w = (double)acd.controllers[0].rot_quat[3];
            
            // just making these as vector objects incase i want to do some vector math
            MVector p_vec;
            p_vec.x = p_x;
            p_vec.y = p_y;
            p_vec.z = p_z;
            
            // just making these as vector objects incase i want to do some vector math
            MVector q_vec;
            q_vec.x = q_x;
            q_vec.y = q_y;
            q_vec.z = q_z;
            
            
            double buttonData = 0.0;
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_1 ) ) {
                buttonData += .001;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_2 ) ) {
                buttonData += .01;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_3 ) ) {
                buttonData += .1;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_4 ) ) {
                buttonData += 1;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_START ) ) {
                buttonData += 10;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_BUMPER ) ) {
                buttonData += 100;
            }
            if( left_states.buttonJustPressed( SIXENSE_BUTTON_JOYSTICK ) ) {
                buttonData += 1000;
            }
             
            MQuaternion quat;
            quat.x = q_vec.x;
            quat.y = q_vec.y;
            quat.z = q_vec.z;
            quat.w = q_w;
            
            // resolve as xyz euler rotations
            MEulerRotation euler = quat.asEulerRotation();
            
            euler.reorderIt(MEulerRotation::kZXY);
            MVector er_vec = euler.asVector();
            

            double* doubleData = reinterpret_cast<double*>(buffer.ptr());
            
			doubleData[0] = (double)p_vec.x * world_scale; 
			doubleData[1] = (double)p_vec.y * world_scale; //*2; // for some reason y is squished? 
			doubleData[2] = (double)p_vec.z * world_scale;
             
            doubleData[3] = (double)er_vec.x * (180/pi); 
			doubleData[4] = (double)er_vec.y * (180/pi); 
 			doubleData[5] = (double)er_vec.z * (180/pi); 
            
            
            doubleData[6] = (double)acd.controllers[0].trigger;
            doubleData[7] = buttonData;
            doubleData[8] = (double)acd.controllers[0].joystick_x;
            doubleData[9] = (double)acd.controllers[0].joystick_y;
            
			pushThreadData( buffer );
		}
		endThreadLoop();
	}
	setDone( true );
}

void hydraDeviceNode::threadShutdownHandler()
{
	// Stops the loop in the thread handler
	setDone( true );
}

void* hydraDeviceNode::creator()
{
	return new hydraDeviceNode;
}

MStatus hydraDeviceNode::initialize()
{

	MStatus status;
	MFnNumericAttribute numAttr;
    
 
	outputTranslateX = numAttr.create("outputTranslateX", "otx", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputTranslateX");
	outputTranslateY = numAttr.create("outputTranslateY", "oty", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputTranslateY");
	outputTranslateZ = numAttr.create("outputTranslateZ", "otz", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputTranslateZ");
	outputTranslate = numAttr.create("outputTranslate", "ot", outputTranslateX, outputTranslateY, 
									 outputTranslateZ, &status);
	MCHECKERROR(status, "create outputTranslate");
    
	outputRotateX = numAttr.create("outputRotateX", "orx", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputRotateX");
	outputRotateY = numAttr.create("outputRotateY", "ory", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputRotateY");
	outputRotateZ = numAttr.create("outputRotateZ", "orz", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputRotateZ");
	outputRotate = numAttr.create("outputRotate", "or", outputRotateX, outputRotateY, 
									 outputRotateZ, &status);
	MCHECKERROR(status, "create outputRotate");
    
    
	offsetTranslateX = numAttr.create("offsetTranslateX", "oftx", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetTranslateX");
	offsetTranslateY = numAttr.create("offsetTranslateY", "ofty", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetTranslateY");
	offsetTranslateZ = numAttr.create("offsetTranslateZ", "oftz", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetTranslateZ");
	offsetTranslate = numAttr.create("offsetTranslate", "oft", offsetTranslateX, offsetTranslateY, 
                                  offsetTranslateZ, &status);
	MCHECKERROR(status, "create offsetTranslate");
    
	offsetRotateX = numAttr.create("offsetRotateX", "ofrx", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetRotateX");
	offsetRotateY = numAttr.create("offsetRotateY", "ofry", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetRotateY");
	offsetRotateZ = numAttr.create("offsetRotateZ", "ofrz", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create offsetRotateZ");
	offsetRotate = numAttr.create("offsetRotate", "ofr", offsetRotateX, offsetRotateY, 
                                     offsetRotateZ, &status);
	MCHECKERROR(status, "create offsetRotate");

    outputTrigger = numAttr.create("outputTrigger", "otrg", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputTrigger");
    
    outputButtons = numAttr.create("outputButtons", "obtn", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputButtons");
    
    outputJoystickX = numAttr.create("outputJoystickX", "ojsx", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputJoystickX");
    outputJoystickY = numAttr.create("outputJoystickY", "ojsy", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputJoystickY");
    outputJoystickZ = numAttr.create("outputJoystickZ", "ojsz", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputJoystickZ");
	outputJoystick = numAttr.create("outputJoystick", "ojs", outputJoystickX, outputJoystickY, outputJoystickZ, &status);
    MCHECKERROR(status, "create offsetJoystick");
    
    
    zoomX = numAttr.create("zoomX", "zx", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomX");
    
    zoomY = numAttr.create("zoomY", "zy", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomY");

    zoomSpeedX = numAttr.create("zoomSpeedX", "zsx", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomSpeedX");
    
    zoomSpeedY = numAttr.create("zoomSpeedY", "zsy", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomSpeedY");
    

	ADD_ATTRIBUTE(outputTranslate);
	ADD_ATTRIBUTE(outputRotate);
    ADD_ATTRIBUTE(offsetTranslate);
    ADD_ATTRIBUTE(offsetRotate);
    ADD_ATTRIBUTE(outputTrigger);
    ADD_ATTRIBUTE(outputButtons);
    ADD_ATTRIBUTE(outputJoystick);
    ADD_ATTRIBUTE(zoomX);
    ADD_ATTRIBUTE(zoomY);
    ADD_ATTRIBUTE(zoomSpeedX);
    ADD_ATTRIBUTE(zoomSpeedY);
    
	ATTRIBUTE_AFFECTS( live, outputTranslate);
	ATTRIBUTE_AFFECTS( frameRate, outputTranslate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputRotate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputTrigger);
    ATTRIBUTE_AFFECTS( outputTranslate, outputButtons);
    ATTRIBUTE_AFFECTS( outputTranslate, outputJoystick);
    ATTRIBUTE_AFFECTS( outputTranslate, zoomX);
    ATTRIBUTE_AFFECTS( outputTranslate, zoomY);

	return MS::kSuccess;
}

MStatus hydraDeviceNode::compute( const MPlug& plug, MDataBlock& block )
{
	MStatus status;
	if( plug == output || plug == outputTranslate || plug == outputTranslateX ||
       plug == outputTranslateY || plug == outputTranslateZ ||
       plug == outputRotate || plug == outputRotateX ||
       plug == outputRotateY || plug == outputRotateZ)
	{
		MCharBuffer buffer;
		if ( popThreadData(buffer) )
		{
			double* doubleData = reinterpret_cast<double*>(buffer.ptr());
            

			MDataHandle outputTranslateHandle = block.outputValue( outputTranslate, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputTranslate");
            
			MDataHandle outputRotateHandle = block.outputValue( outputRotate, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputRotate");

            MDataHandle outputTriggerHandle = block.outputValue( outputTrigger, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputTrigger");
        
            MDataHandle outputButtonsHandle = block.outputValue( outputButtons, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputButtons");

            MDataHandle outputJoystickHandle = block.outputValue( outputJoystick, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputJoystick");
            
            MDataHandle zoomXHandle = block.outputValue( zoomX, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomXHandle");
			
            MDataHandle zoomSpeedXHandle = block.outputValue( zoomSpeedX, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomSpeedXHandle");
            
            MDataHandle zoomYHandle = block.outputValue( zoomY, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomYHandle");
			
            MDataHandle zoomSpeedYHandle = block.outputValue( zoomSpeedY, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomSpeedYHandle");            
            
			double3& outputTranslate = outputTranslateHandle.asDouble3();
			outputTranslate[0] = (double)doubleData[0];
			outputTranslate[1] = (double)doubleData[1];
			outputTranslate[2] = (double)doubleData[2];
            
			double3& outputRotate = outputRotateHandle.asDouble3();
			outputRotate[0] = (double)doubleData[3];
			outputRotate[1] = (double)doubleData[4];
			outputRotate[2] = (double)doubleData[5];
            
            double& outputTrigger = outputTriggerHandle.asDouble();
            outputTrigger = (double)doubleData[6];
            
            double& outputButtons = outputButtonsHandle.asDouble();
            outputButtons = (double)doubleData[7];
            
			double3& outputJoystick = outputJoystickHandle.asDouble3();
			outputJoystick[0] = (double)doubleData[8];
			outputJoystick[1] = (double)doubleData[9];
			outputJoystick[2] = 0;
            
            double& zoomSpeedX = zoomSpeedXHandle.asDouble();
            
            double& zoomX = zoomXHandle.asDouble();
            zoomX = zoomX + (doubleData[8] * (zoomX/35 * zoomSpeedX));
            
            double& zoomSpeedY = zoomSpeedYHandle.asDouble();
            
            double& zoomY = zoomYHandle.asDouble();
            zoomY = zoomY + (doubleData[9] * (zoomY/35 * zoomSpeedY));
            
			block.setClean( plug );
            
			releaseDataStorage(buffer);
			return ( MS::kSuccess );
		}
		else
		{
			return MS::kFailure;
		}
	}
    
	return ( MS::kUnknownParameter );
}

MStatus initializePlugin( MObject obj )
{
	MStatus status;
	MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");

	status = plugin.registerNode( "hydraDevice", 
								  hydraDeviceNode::id,
								  hydraDeviceNode::creator,
								  hydraDeviceNode::initialize,
								  MPxNode::kThreadedDeviceNode );
	if( !status ) {
		status.perror("failed to registerNode hydraDeviceNode");
	}

	return status;
}

MStatus uninitializePlugin( MObject obj )
{
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode( hydraDeviceNode::id );
	if( !status ) {
		status.perror("failed to deregisterNode hydraDeviceNode");
	}

	return status;
}




