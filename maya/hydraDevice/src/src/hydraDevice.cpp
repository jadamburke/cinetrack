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
            
            //printf ("%s \n", "Getting Data");
            sixenseSetActiveBase(0);
            sixenseAllControllerData acd;
            sixenseGetAllNewestData( &acd );
            sixenseUtils::getTheControllerManager()->update( &acd );
            
            

            
            double m_x = (double)acd.controllers[0].rot_quat[0];
            double m_y = (double)acd.controllers[0].rot_quat[1];
            double m_z = (double)acd.controllers[0].rot_quat[2];
            double m_w = (double)acd.controllers[0].rot_quat[3];
            
            //MQuaternion	(	double 	xx,yy,zz,ww )	
            MQuaternion quat;
            quat.x = m_x;
            quat.y = m_y;
            quat.z = m_z;
            quat.w = m_w;
            MVector vec = quat.asEulerRotation().asVector();
            const double pi = 3.14159265358979323846;

            // found algorithm for quaternion to zxy rotation order which provides ideal euler angles for 
            // camera systems wherby roll (z) is the least likely to rotate more than 90 degrees
            
            // for some reason the yaw is skewed towards the base station, might need to use the position
            // vector to compensate the quaternion vector
            
            /*
            double yaw;
            double pitch;
            double roll;
            
            const double w2 = m_w*m_w;
            const double x2 = m_x*m_x;
            const double y2 = m_y*m_y;
            const double z2 = m_z*m_z;
            const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
            const double abcd = m_w*m_x + m_y*m_z;
            const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
            const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
            if (abcd > (0.5-eps)*unitLength)
            {
                yaw = 2 * atan2(m_y, m_w);
                pitch = pi;
                roll = 0;
            }
            else if (abcd < (-0.5+eps)*unitLength)
            {
                yaw = -2 * ::atan2(m_y, m_w);
                pitch = -pi;
                roll = 0;
            }
            else
            {
                const double adbc = m_w*m_z - m_x*m_y;
                const double acbd = m_w*m_y - m_x*m_z;
                yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
                pitch = ::asin(2*abcd/unitLength);
                roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
            }
           
            yaw *= (180/pi);
            pitch *= (180/pi);
            roll *= (180/pi);
            
            
            // rotation order = zxy zyx
            
            // yaw
            //double rotx = atan2(2*((m_w * m_x) + (m_y * m_z)), 1 - (2 * ((m_x* m_x) + (m_y * m_y)))) * (180/3.141596);
            // pitch
            //double roty = asin(2 * ((m_w * m_y) - (m_z * m_x))) * (180/3.141596);
            // roll
            //double rotz = atan2(2 * ((m_w * m_z) + (m_x * m_y)), 1 - (2 * ((m_y * m_y) + (m_z * m_z)))) * (180/3.141596);
            
            /*
            // yaw
            double rotx = atan2(2*((m_w * m_x) + (m_y * m_z)), 1 - (2 * ((m_x* m_x) + (m_y * m_y)))) * (180/3.141596);
            // pitch
            double roty = asin(2 * ((m_w * m_y) - (m_z * m_x))) * (180/3.141596);
            // roll
            double rotz = atan2(2 * ((m_w * m_z) + (m_x * m_y)), 1 - (2 * ((m_y * m_y) + (m_z * m_z)))) * (180/3.141596);     
             */       
            double* doubleData = reinterpret_cast<double*>(buffer.ptr());
			doubleData[0] = (double)acd.controllers[0].pos[0] *.01; 
			doubleData[1] = (double)acd.controllers[0].pos[1] *.01; 
			doubleData[2] = (double)acd.controllers[0].pos[2] *.01;
            
			//doubleData[3] = (double)pitch; 
			//doubleData[4] = (double)yaw; 
			//doubleData[5] = (double)roll; 
            doubleData[3] = (double)vec.x * (180/pi); 
			doubleData[4] = (double)vec.y * (180/pi); 
 			doubleData[5] = (double)vec.z * (180/pi); 
            
            
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




