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
 
 recordAttr -at "translateX" -at "translateY" -at "translateZ";
 recordAttr -at "rotateX" -at "rotateY" -at "rotateZ";
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
	static MObject 		outputQuatVec;
	static MObject 		outputQuatVecX;
	static MObject 		outputQuatVecY;
	static MObject 		outputQuatVecZ;
    static MObject      outputQuatW;

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
MObject hydraDeviceNode::outputQuatVec;
MObject hydraDeviceNode::outputQuatVecX;
MObject hydraDeviceNode::outputQuatVecY;
MObject hydraDeviceNode::outputQuatVecZ;
MObject hydraDeviceNode::outputQuatW;

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
            
			double p_x = (double)acd.controllers[0].pos[0]; 
			double p_y = (double)acd.controllers[0].pos[1]; 
			double p_z = (double)acd.controllers[0].pos[2];   
            acd.controllers[0].hemi_tracking_enabled = 1;
            
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
			doubleData[1] = (double)p_vec.y * world_scale*2; // for some reason y is squished? 
			doubleData[2] = (double)p_vec.z * world_scale;
             
            doubleData[3] = (double)er_vec.x * (180/pi); 
			doubleData[4] = (double)er_vec.y * (180/pi); 
 			doubleData[5] = (double)er_vec.z * (180/pi); 
            
            
            doubleData[6] = (double)acd.controllers[0].rot_quat[0];
            doubleData[7] = (double)acd.controllers[0].rot_quat[1];
            doubleData[8] = (double)acd.controllers[0].rot_quat[2];
            doubleData[9] = (double)acd.controllers[0].rot_quat[3];
            
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
    
    
	outputQuatVecX = numAttr.create("outputQuatVecX", "oqvx", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputQuatVecX");
	outputQuatVecY = numAttr.create("outputQuatVecY", "oqvy", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputQuatVecY");
	outputQuatVecZ = numAttr.create("outputQuatVecZ", "oqvz", MFnNumericData::kDouble, 0.0, &status);
	MCHECKERROR(status, "create outputQuatVecZ");
	outputQuatVec = numAttr.create("outputQuatVec", "oqv", outputQuatVecX, outputQuatVecY, 
                                  outputQuatVecZ, &status);
	MCHECKERROR(status, "create outputQuatVec");

    outputQuatW = numAttr.create("outputQuatW", "oqw", MFnNumericData::kDouble, 0.0, &status);
    MCHECKERROR(status, "create outputQuatW");

	ADD_ATTRIBUTE(outputTranslate);
	ADD_ATTRIBUTE(outputRotate);
    ADD_ATTRIBUTE(outputQuatVec);
    ADD_ATTRIBUTE(outputQuatW);
    
	ATTRIBUTE_AFFECTS( live, outputTranslate);
	ATTRIBUTE_AFFECTS( frameRate, outputTranslate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputRotate);
    ATTRIBUTE_AFFECTS( outputTranslate, outputQuatVec);
    ATTRIBUTE_AFFECTS( outputTranslate, outputQuatW);

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

            MDataHandle outputQuatVecHandle = block.outputValue( outputQuatVec, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputRotate");
        
            MDataHandle outputQuatWHandle = block.outputValue( outputQuatW, &status );
            MCHECKERROR(status, "Error in block.outputValue for outputRotate");
			
            
			double3& outputTranslate = outputTranslateHandle.asDouble3();
			outputTranslate[0] = (double)doubleData[0];
			outputTranslate[1] = (double)doubleData[1];
			outputTranslate[2] = (double)doubleData[2];
            
			double3& outputRotate = outputRotateHandle.asDouble3();
			outputRotate[0] = (double)doubleData[3];
			outputRotate[1] = (double)doubleData[4];
			outputRotate[2] = (double)doubleData[5];
            
			double3& outputQuatVec = outputQuatVecHandle.asDouble3();
			outputQuatVec[0] = (double)doubleData[6];
			outputQuatVec[1] = (double)doubleData[7];
			outputQuatVec[2] = (double)doubleData[8];
            
            double& outputQuatW = outputQuatWHandle.asDouble();
            outputQuatW = (double)doubleData[9];
            
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




