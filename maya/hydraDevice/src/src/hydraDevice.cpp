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
 
/Users/adam/Documents/maya/projects/default/scenes/hydraTest2_recorded.ma

	setAttr ( $node + ".live" ) 1;
 
 
 $r = `createNode record`;
 connectAttr null1.hydraTranslateX ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraTranslateY ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraTranslateZ ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraRotateX ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraRotateY ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraRotateZ ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraZoomX ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraZoomY ($r+".input");
 $r = `createNode record`;
 connectAttr null1.hydraZoomZ ($r+".input");
 
 play -rec;
 
 
 // create a script job to watch for button events on the hydraDevice
 global proc hydraButtonWatch(){
 float $btnVal = `getAttr hydraDevice1.outputButtons`;
 if ($btnVal != 0){
 // do rehersal pass (no recording)
 if ($btnVal == 0.01){
 currentTime -e `playbackOptions -q -min`;
 if (`play -q -st` == true){
 play -st false;
 }
 else {
 playbackOptions -loop "once";
 play -st true;
 playbackOptions -loop "continuous";
 }
 }
 // do recording pass
 else if ($btnVal == 0.1){
 // backup current take data
 // duplicate anim curve and transform
 // rename duplicates
 // connect them together
 
 currentTime -e `playbackOptions -q -min`;
 
 // set update to single instead of all
 $pauseLength = 3;
 for ($i=0;$i<$pauseLength;$i++){
 pause -sec 1;
 inViewMessage -smg ("Ready"+$i) -pos midCenter -fontSize 28 -bkc 0x00000000 -fade;
 }
 
 
 play -rec;
 }
 print ($btnVal+" button value!\n");
 }
 
 }// end proc
 scriptJob -runOnce false  -killWithScene -attributeChange hydraDevice1.outputButtons hydraButtonWatch;
 
 
 // creates a copy of the hydraAnimData null along with animation curves
 global proc sixenseDuplicateTakeRecNode (){
 // find the active data
 // find the animation cuves connected to data
 // duplicate them
 // rename them to a standardized naming system
 
 }
 
 global proc sixenseChangeTake (string $takeRecNode){
 // connects the active takeRecNode attrs to the specified takeRecNode.
 // check to see if it has valid attribute & if they are animated
 //
 
 }
 
 // creates an active take and connects it to the hydra device
 // also attaches the record nodes
 global proc sixenseCreateActiveTake () {
 
 }
 
 global proc sixenseRemoveAllRecordNodes (){
 
 
 
 }

*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>


#include <maya/MFnPlugin.h>
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MEulerRotation.h>
#include <maya/MAnimControl.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnCamera.h>
#include <maya/MSelectionList.h>

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

#include <maya/MTimerMessage.h>
#include <maya/MConditionMessage.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MGlobal.h>


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

// hydraRecord Callback ID data store
MCallbackIdArray callbackIds;
int callbackCounter;
double gStartClock;

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
    static MStatus      togglePlayback();

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
    static MObject      zoom;
    static MObject      zoomX;
    static MObject      zoomY;
    static MObject      zoomZ;
    static MObject      zoomSpeed;
    static MObject      zoomSpeedX;
    static MObject      zoomSpeedY;
    static MObject      zoomSpeedZ;

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
MObject hydraDeviceNode::zoom;
MObject hydraDeviceNode::zoomX;
MObject hydraDeviceNode::zoomY;
MObject hydraDeviceNode::zoomZ;
MObject hydraDeviceNode::zoomSpeed;
MObject hydraDeviceNode::zoomSpeedX;
MObject hydraDeviceNode::zoomSpeedY;
MObject hydraDeviceNode::zoomSpeedZ;



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
	createMemoryPools( 2, 10, sizeof(double));
    
    //createMemoryPools (12, 3, sizeof(Ptr)) ;
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

    //sixenseSetFilterEnabled(1);
    //sixenseSetFilterParams(.2, 0, 1, .1);
    static FILE *log_file = 0;
    log_file = fopen( "sixense_log.txt", "w" );
    
    
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
            clock();
            
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
			doubleData[1] = (double)p_vec.y * world_scale;
			doubleData[2] = (double)p_vec.z * world_scale;
             
            doubleData[3] = (double)er_vec.x * (180/pi); 
			doubleData[4] = (double)er_vec.y * (180/pi); 
 			doubleData[5] = (double)er_vec.z * (180/pi); 
            
            
            doubleData[6] = (double)acd.controllers[0].trigger;
            doubleData[7] = buttonData;
            doubleData[8] = (double)acd.controllers[0].joystick_x;
            doubleData[9] = (double)acd.controllers[0].joystick_y;
            
            // log the data to a temp file
            if( log_file ) {
                //fprintf( log_file, "base: %d controller: %d ", 0, 0 );
                
                fprintf( log_file, "%f %f %f %f %f %f %f %f %f %f %f\n", (double)clock(),doubleData[0],doubleData[1],doubleData[2],doubleData[3],doubleData[4],doubleData[5],doubleData[6],doubleData[7],doubleData[8],doubleData[9]);
            }
            
            
			pushThreadData( buffer );
		}
		endThreadLoop();
	}
	setDone( true );
    fclose(log_file);
}

MStatus togglePlayback()
{
    MStatus status;
        MAnimControl anim;
        if (anim.isPlaying()){
            status = anim.stop();
        }
        else{
            status = anim.playForward();
        }
    return status;
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
    zoomZ = numAttr.create("zoomZ", "zz", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomZ");
	zoom = numAttr.create("zoom", "z", zoomX, zoomY, zoomZ, &status);
    MCHECKERROR(status, "create zoom");

    zoomSpeedX = numAttr.create("zoomSpeedX", "zsx", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomSpeedX");
    zoomSpeedY = numAttr.create("zoomSpeedY", "zsy", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomSpeedY");
    zoomSpeedZ = numAttr.create("zoomSpeedZ", "zsz", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create zoomSpeedZ");
	zoomSpeed = numAttr.create("zoomSpeed", "zs", zoomSpeedX, zoomSpeedY, zoomSpeedZ, &status);
    MCHECKERROR(status, "create zoomSpeed");
    

	ADD_ATTRIBUTE(outputTranslate);
	ADD_ATTRIBUTE(outputRotate);
    ADD_ATTRIBUTE(offsetTranslate);
    ADD_ATTRIBUTE(offsetRotate);
    ADD_ATTRIBUTE(outputTrigger);
    ADD_ATTRIBUTE(outputButtons);
    ADD_ATTRIBUTE(outputJoystick);
    ADD_ATTRIBUTE(zoom);
    ADD_ATTRIBUTE(zoomSpeed);
    
	ATTRIBUTE_AFFECTS( live, outputTranslate);
	ATTRIBUTE_AFFECTS( frameRate, outputTranslate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputRotate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputTrigger);
    ATTRIBUTE_AFFECTS( outputTranslate, outputButtons);
    ATTRIBUTE_AFFECTS( outputTranslate, outputJoystick);
    ATTRIBUTE_AFFECTS( outputTranslate, zoom);
    


	return MS::kSuccess;
}

MStatus hydraDeviceNode::compute( const MPlug& plug, MDataBlock& block ) {
	MStatus status;
	if( plug == output || plug == outputTranslate || plug == outputTranslateX ||
       plug == outputTranslateY || plug == outputTranslateZ ||
       plug == outputRotate || plug == outputRotateX ||
       plug == outputRotateY || plug == outputRotateZ)
    {
		MCharBuffer buffer;

		if ( popThreadData(buffer) ) {
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
            
            MDataHandle zoomHandle = block.outputValue( zoom, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomHandle");
			
            MDataHandle zoomSpeedHandle = block.outputValue( zoomSpeed, &status );
            MCHECKERROR(status, "Error in block.outputValue for zoomSpeedHandle");
                     
            
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
            
            double3& zoomSpeed = zoomSpeedHandle.asDouble3();
            
            double3& zoom = zoomHandle.asDouble3();
            zoom[0] = zoom[0] + (doubleData[8] * (zoom[0]/35 * zoomSpeed[0]));
            zoom[1] = zoom[1] + (doubleData[9] * (zoom[1]/35 * zoomSpeed[1]));            
            zoom[2] = 0;
            
            
			block.setClean( plug );
            
			releaseDataStorage(buffer);

            // BUTTON PRESS HANDLING (THIS IS SHITTY WAY TO DO THIS, BUT WORKS)
            if (outputButtons == 0.001){
                printf ("%s \n", "Button 1 Press Detected.");
                MStatus status;
                MAnimControl anim;
                const MTime minTime = anim.minTime();
                if (anim.isPlaying()){
                    status = anim.stop();
                }
                else{
                    anim.setCurrentTime(minTime);
                    status = anim.playForward();
                }
            }
            else if (outputButtons == 0.01){
                printf ("%s \n", "Button 2 Press Detected.");
                MStatus status;
                MAnimControl anim;
                status = anim.stop();
                
            }
            else if (outputButtons == 0.1){
                printf ("%s \n", "Button 3 Press Detected.");
                MString melCmd = "hydraRecord";
                MGlobal::executeCommand( melCmd );
            }
            else if (outputButtons == 1){
                printf ("%s \n", "Button 4 Press Detected.");
                
            }
            else if (outputButtons == 10){
                printf ("%s \n", "Button Start Press Detected.");
                
            }
            else if (outputButtons == 100){
                printf ("%s \n", "Button Bumper Press Detected.");
                
            }
            else if (outputButtons == 1000){
                printf ("%s \n", "Button Joystick Press Detected.");

            }
  
			return ( MS::kSuccess );
		}
		else {
			return MS::kFailure;
		}
	}
    
	return ( MS::kUnknownParameter );
}

// PLUGIN CUSTOM COMMANDS START HERE


static void hydraPreRollCB(float elapsedTime, float lastTime, void * data){
    
    printf ("%s \n", "hydraPreRollCB Called");
    //int i = (int)(size_t)data;
    int count = (int)(size_t)data;
    MAnimControl anim;
    
    count =  count - 1;
    callbackCounter--;
    double asStr = (double)callbackCounter;
    data = (void *)(size_t)count;


    
    // show a view message stating the time left until record
    printf ("elaps:%f, last:%f, count:%d\n", elapsedTime,lastTime,callbackCounter);
    if (callbackCounter == 0){
        MString melCmd = "inViewMessage -smg (\"GO\") -fst 1 -fot 1 -pos midCenter -fontSize 38 -bkc 0x00000000 -fade;";
        MGlobal::executeCommand( melCmd );
        
        
        melCmd = "hydraCamFromLog hydraCamFromLog -l \"sixense_data.txt\" -sc ";
        gStartClock = clock();
        melCmd += gStartClock;
        //melCmd = "play -rec";
        //MGlobal::executeCommand( melCmd );
        
        anim.playForward();
        printf("CLOCK START TICK: %f",gStartClock);
    }
    else if (callbackCounter > 0){
        MString melCmd = "inViewMessage -smg (\"Ready ";
        melCmd += asStr;
        melCmd +="\") -fst 100 -fot 100 -pos midCenter -fontSize 38 -bkc 0x00000000 -fade;";
        MGlobal::executeCommand( melCmd );
    }
}

// called when playback has stopped after doing a record play
static void hydraPostRecordCallback(bool state, void * data){
    printf ("%s \n", "playbackCB callback called");
    MAnimControl anim;
    MString melCmd;
    if (state == false){
        printf ("%s \n", "Playback State False");
        melCmd = "inViewMessage -smg (\"Finish\") -fst 100 -fot 100 -pos midCenter -fontSize 38 -bkc 0x00000000 -fade;";
        MGlobal::executeCommand( melCmd );
        anim.setPlaybackMode(MAnimControl::kPlaybackLoop);

        // read the log file and make a take camera for it
        melCmd = "hydraCamFromLog hydraCamFromLog -l \"sixense_data.txt\" -sc ";
        melCmd += gStartClock;
        MGlobal::executeCommand( melCmd );
        // remove all callbacks
        for (unsigned int i=0; i < callbackIds.length(); i++ ) {
            MMessage::removeCallback( (MCallbackId)callbackIds[i] );
        }
    }
    else{
        printf ("%s \n", "Playback State True");
    }


}





/////////////////////////////////////////
// RUNS A RECORD METHOD WITH A PREROLL //
/////////////////////////////////////////

class hydraRecord : public MPxCommand
{
public:
    MTimerMessage prerolltimer;
    MConditionMessage recordFinished;
    MStatus doIt( const MArgList& args );
    bool isUndoable() const;
    static void* creator();
};

MStatus hydraRecord::doIt( const MArgList& args ) {
    
    MCallbackId id;
    
    //cout << "Hello " << args.asString( 0 ).asChar() << endl;
    printf ("%s \n", "hydraRecord::doIt() called");
    //MGlobal::executeCommand
    //MTimerMessage prerolltimer;
    MStatus status;
    const MString condition = "playingBack";
    
    // set the playback head to the playback start time
    MAnimControl anim;
    anim.setPlaybackMode(MAnimControl::kPlaybackOnce);
    const MTime minTime = anim.minTime();
    anim.setCurrentTime(minTime);
    int c = 3;
    
    // do 3 second preroll;
    //double count = 3;
    //data = &count;
    // register a timer callback to call itself 3 times (may not be accurate)
    
    callbackCounter = 3;
    
    id = prerolltimer.addTimerCallback(1, hydraPreRollCB, (void *)(size_t)c, &status);
    
    if (status) {
        callbackIds.append( id );
        printf ("%s \n", "timer callback added");
        MString melCmd = "inViewMessage -smg (\"Ready 3\") -fst 100 -fot 100 -pos midCenter -fontSize 38 -bkc 0x00000000 -fade;";
        MGlobal::executeCommand( melCmd );
    }
    else {
        printf ("%s \n", "timer callback FAILED");
    }

    // create a new playbackFinished callback
    id = recordFinished.addConditionCallback(condition, hydraPostRecordCallback,&status);
    if (status) {
        callbackIds.append( id );
        printf ("%s \n", "record callback added");
    }
    else {
        printf ("%s \n", "record callback FAILED");
    }
    
    printf ("%s \n", "hydraRecord::doIt() finished");
    return MS::kSuccess;
}

bool hydraRecord::isUndoable() const {
    return false;
}

void* hydraRecord::creator() {
    return new hydraRecord;
}





///////////////////////////////////////////////////////////////////////
// CREATES A MAYA CAMERA WITH ANIMATION CURVES FROM A HYDRA LOG FILE //
///////////////////////////////////////////////////////////////////////

class hydraCamFromLog : public MPxCommand
{
public:
    MStatus doIt( const MArgList& args );
    MStatus redoIt();
    MStatus undoIt();
    bool isUndoable() const;
    static void* creator();
private:
    MDagPath fDagPath;
    MString logPath;
    MTime startFrame;
    MTime endFrame;
    double durationFrames;
    double startClock;
    double endClock;
    MStringArray logData;
};

MStatus hydraCamFromLog::doIt( const MArgList& args ) {
    MStatus status;
    for ( int i = 0; i < args.length(); i++ ){
        if ( MString( "-l" ) == args.asString( i, &status ) && MS::kSuccess == status ){
            MString tmp = args.asString( ++i, &status );
            if ( MS::kSuccess == status )
                logPath = tmp;
        }
        else if ( MString( "-sc" ) == args.asString( i, &status ) && MS::kSuccess == status ){
            double tmp = args.asDouble( ++i, &status );
            if ( MS::kSuccess == status )
                startClock = tmp;
        }
        else if ( MString( "-ec" ) == args.asString( i, &status ) && MS::kSuccess == status ){
            double tmp = args.asDouble( ++i, &status );
            if ( MS::kSuccess == status )
                endClock = tmp;
        }
        else{
            MString msg = "Invalid flag: ";
            msg += args.asString( i );
            displayError( msg );
            return MS::kFailure;
        }
    }
    
    
    // load the log file into a string array (limit this to 10,000 entries)
    // need to have error handling here
    printf ("%s \n", "Reading Log File");
    char buffer [200];
    static FILE *log_file = 0;
    log_file = fopen( "sixense_log.txt", "r" );
    printf ("%s \n", "Log File Opened");
    if( log_file ) {
        while(!feof(log_file)) {
        // read the file into memory
        fgets (buffer,200,log_file);
        logData.append(buffer);
        //fscanf( log_file, "%d %f %f %f %f %f %f %f %f %f %f\n", (double)clock(),doubleData[0],doubleData[1],doubleData[2],doubleData[3],doubleData[4],doubleData[5],doubleData[6],doubleData[7],doubleData[8],doubleData[9]);
        }
        fclose(log_file);
    }
    printf ("%s \n", "FINISHED Reading Log File");
    
    
    // store the start and end frames to generate keys for
    MAnimControl anim;
    startFrame = anim.minTime();
    endFrame = anim.maxTime();
    durationFrames = endFrame.value() - startFrame.value();
    
    // if no endClock argument is specified then assume its the duration of the playback range
    if (endClock == 0){
        endClock = startClock + ((durationFrames / 24)*CLOCKS_PER_SEC);
    }
    printf ("%s \n", "Calling redoIt()");
    redoIt();
    
    return MS::kSuccess;
}

MStatus hydraCamFromLog::redoIt() {
    
    MStatus status;
    
    // process the log data and input values and generate a maya camera with animation curves

    
    // create the camera
    //MDagPath mObject;
    //MFnDagNode fnSet( mObject, &status );
    
    MObject mGroup;
    const MString groupName = "hydraGroup";
    MSelectionList tempList;
    tempList.add(groupName);
    tempList.getDependNode(0, mGroup);

    // Create Camera parented under hydraGroup
    MString camName = "hydraCam_Take";
    MFnCamera hydCamFn;
    const MObject newCam = hydCamFn.create(mGroup);
    MDagPath mObject;
    MDagPath::getAPathTo(newCam,mObject);
    //mObject.getAPathTo(newCam);
    MFnDagNode fnSet( mObject, &status );
    fnSet.setName(camName);
    
    
    if ( MS::kSuccess != status ) {
        cerr << "Failure to create function set\n";
    }
    
    
    // create the translate anim curves from the data
    MString attrName( "translateX" );
    const MObject attrX = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetX;
    acFnSetX.create( mObject.transform(), attrX, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (translateX)\n";
    }
    attrName = "translateY";
    const MObject attrY = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetY;
    acFnSetY.create( mObject.transform(), attrY, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (translateY)\n";
    }
    attrName = "translateZ";
    const MObject attrZ = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetZ;
    acFnSetZ.create( mObject.transform(), attrZ, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (translateZ)\n";
    }
    
    // create the rotation anim curves from the data
    attrName = "rotateX";
    const MObject attrRX = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetRX;
    acFnSetRX.create( mObject.transform(), attrRX, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (rotateX)\n";
    }
    attrName = "rotateY";
    const MObject attrRY = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetRY;
    acFnSetRY.create( mObject.transform(), attrRY, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (rotateY)\n";
    }
    attrName = "rotateZ";
    const MObject attrRZ = fnSet.attribute( attrName, &status );
    MFnAnimCurve acFnSetRZ;
    acFnSetRZ.create( mObject.transform(), attrRZ, NULL, &status );
    if ( MS::kSuccess != status ) {
        cerr << "Failure creating MFnAnimCurve function set (rotateZ)\n";
    }
    
    // Build the keyframes

    
    //const unsigned int maxIterations = 1000;

    // seek start clock time in the buffer
    unsigned int logLineCount = logData.length();
    
    // for each line in the log buffer
    printf ("%s length=%d\n", "Starting loop of log buffer",logLineCount);
    for( int i = 0; i <= logLineCount; i++ ) {
        MStringArray strVals;
        logData[i].split(' ', strVals);
        //printf ("%f, ",strVals[0].asDouble());
        
        if (strVals[0].asDouble()>startClock){
            printf ("Found clock=%f startClock=%f\n", strVals[0].asDouble(),startClock);
            printf("tx:%f ty:%f tz:%f rx:%f ry:%f rz:%f",strVals[1].asDouble(),strVals[2].asDouble(),strVals[3].asDouble(),strVals[4].asDouble(),strVals[5].asDouble(),strVals[6].asDouble());

            // map clock tick to maya timeline
            double clockDiff = difftime(strVals[0].asDouble(),startClock);
            double frame = startFrame.value()+((clockDiff/CLOCKS_PER_SEC)*24); // need to get frame rate from sys

            // make this process the log data
            double tx = strVals[1].asDouble();
            double ty = strVals[2].asDouble();
            double tz = strVals[3].asDouble();
            MAngle rx,ry,rz;
            rx.setUnit(MAngle::kDegrees);
            ry.setUnit(MAngle::kDegrees);
            rz.setUnit(MAngle::kDegrees);
            rx.setValue(strVals[4].asDouble());
            ry.setValue(strVals[5].asDouble());
            rz.setValue(strVals[6].asDouble());
        
        
            MTime tm(frame, MTime::kFilm );
            if ( ( MS::kSuccess != acFnSetX.addKeyframe( tm, tx ) ) ||
                ( MS::kSuccess != acFnSetY.addKeyframe( tm, ty ) ) ||
                ( MS::kSuccess != acFnSetZ.addKeyframe( tm, tz ) ) ||
                ( MS::kSuccess != acFnSetRX.addKeyframe( tm, rx.asRadians() ) ) ||
                ( MS::kSuccess != acFnSetRY.addKeyframe( tm, ry.asRadians() ) ) ||
                ( MS::kSuccess != acFnSetRZ.addKeyframe( tm, rz.asRadians() ) ) ) {
                cerr << "Error setting the keyframe\n";
            }
        }
        else if (strVals[0].asDouble() > endClock){
            break;
        }
    }
    
    
    return MS::kSuccess;
}

MStatus hydraCamFromLog::undoIt() {
    
    return MS::kSuccess;
}

bool hydraCamFromLog::isUndoable() const {
    return true;
}

void* hydraCamFromLog::creator() {
    return new hydraCamFromLog;
}



/////////////////////////
// PLUGIN REGISTRATION //
/////////////////////////

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
    
    plugin.registerCommand( "hydraRecord", hydraRecord::creator );
    plugin.registerCommand( "hydraCamFromLog", hydraCamFromLog::creator );

	return status;
}

MStatus uninitializePlugin( MObject obj )
{
	MStatus status;
	MFnPlugin plugin(obj);
    
    // Remove all callbacks
    //
    for (unsigned int i=0; i < callbackIds.length(); i++ ) {
        MMessage::removeCallback( (MCallbackId)callbackIds[i] );
    }

	status = plugin.deregisterNode( hydraDeviceNode::id );
    status = plugin.deregisterCommand("hydraRecord");
    status = plugin.deregisterCommand("hydraCamFromLog");
	if( !status ) {
		status.perror("failed to deregisterNode hydraDeviceNode");
	}
    
    plugin.deregisterCommand( "hydraRecord" );


	return status;
}




