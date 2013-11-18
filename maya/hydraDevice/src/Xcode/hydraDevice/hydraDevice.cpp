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
	loadPlugin randomizerDevice;
	string $node = `createNode randomizerDevice`;
	string $cube[] = `polyCube`;
	connectAttr ( $node + ".outputTranslate" ) ( $cube[0] + ".translate" );
	setAttr ( $node + ".live" ) 1;
*/

#include <stdlib.h>

#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h>

#include <api_macros.h>
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
static bool controller_manager_screen_visible = true;
std::string controller_manager_text_string;


class randomizerDeviceNode : public MPxThreadedDeviceNode
{

public:
						randomizerDeviceNode();
	virtual 			~randomizerDeviceNode();
	
	virtual void		postConstructor();
	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );
	virtual void		threadHandler();
	virtual void		threadShutdownHandler();

	static void*		creator();
	static MStatus		initialize();

public:

	static MObject		outputTranslate;
	static MObject 		outputTranslateX;
	static MObject		outputTranslateY;
	static MObject 		outputTranslateZ;

	static MTypeId		id;

private:
};

MTypeId randomizerDeviceNode::id( 0x00081051 );
MObject randomizerDeviceNode::outputTranslate;
MObject randomizerDeviceNode::outputTranslateX;
MObject randomizerDeviceNode::outputTranslateY;
MObject randomizerDeviceNode::outputTranslateZ;

randomizerDeviceNode::randomizerDeviceNode() 
{

}

randomizerDeviceNode::~randomizerDeviceNode()
{
	destroyMemoryPools();
}

void randomizerDeviceNode::postConstructor()
{
	MObjectArray attrArray;
	attrArray.append( randomizerDeviceNode::outputTranslate );
	setRefreshOutputAttributes( attrArray );

	// we'll be reading one set of translate x,y, z's at a time
	createMemoryPools( 24, 3, sizeof(double));
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

void randomizerDeviceNode::threadHandler()
{
	MStatus status;
	setDone( false );
    
	sixenseInit();
	int curStep = sixenseUtils::getTheControllerManager()->getCurrentStep();
	printf ("curStep :: %d \n", curStep);
	
	//sixenseUtils::getTheControllerManager()->registerSetupCallback( controller_manager_setup_callback );
	//sixenseUtils::getTheControllerManager()->setGameType( sixenseUtils::controller_manager::ONE_PLAYER_TWO_CONTROLLER );
	printf ("curStep :: %d \n", curStep);
	printf ("%s \n", "Sixense Capture Server For Maya2011, version 0.1");
	printf ("%s \n", "Author: Evan Harper, August 2011");

    
    
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
            
            printf ("%s \n", "Getting Data");
            sixenseSetActiveBase(0);
            sixenseAllControllerData acd;
            sixenseGetAllNewestData( &acd );
            sixenseUtils::getTheControllerManager()->update( &acd );
            
            //acd.controllers[0].pos[0];
            //acd.controllers[0].pos[1];
            //acd.controllers[0].pos[2];
            printf ("%f \n", acd.controllers[0].pos[0]);
            
			double* doubleData = reinterpret_cast<double*>(buffer.ptr());
			doubleData[0] = getRandomX() ; doubleData[1] = 0.0; doubleData[2] = 0.0;
			pushThreadData( buffer );
		}
		endThreadLoop();
	}
	setDone( true );
}

void randomizerDeviceNode::threadShutdownHandler()
{
	// Stops the loop in the thread handler
	setDone( true );
}

void* randomizerDeviceNode::creator()
{
	return new randomizerDeviceNode;
}

MStatus randomizerDeviceNode::initialize()
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
	
	ADD_ATTRIBUTE(outputTranslate);

	ATTRIBUTE_AFFECTS( live, outputTranslate);
	ATTRIBUTE_AFFECTS( frameRate, outputTranslate);

	return MS::kSuccess;
}

MStatus randomizerDeviceNode::compute( const MPlug& plug, MDataBlock& block )
{
	MStatus status;
	if( plug == outputTranslate || plug == outputTranslateX ||
		plug == outputTranslateY || plug == outputTranslateZ )
	{
		MCharBuffer buffer;
		if ( popThreadData(buffer) )
		{
			double* doubleData = reinterpret_cast<double*>(buffer.ptr());

			MDataHandle outputTranslateHandle = block.outputValue( outputTranslate, &status );
			MCHECKERROR(status, "Error in block.outputValue for outputTranslate");

			double3& outputTranslate = outputTranslateHandle.asDouble3();
			outputTranslate[0] = doubleData[0];
			outputTranslate[1] = doubleData[1];
			outputTranslate[2] = doubleData[2];

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

	status = plugin.registerNode( "randomizerDevice", 
								  randomizerDeviceNode::id,
								  randomizerDeviceNode::creator,
								  randomizerDeviceNode::initialize,
								  MPxNode::kThreadedDeviceNode );
	if( !status ) {
		status.perror("failed to registerNode randomizerDeviceNode");
	}

	return status;
}

MStatus uninitializePlugin( MObject obj )
{
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode( randomizerDeviceNode::id );
	if( !status ) {
		status.perror("failed to deregisterNode randomizerDeviceNode");
	}

	return status;
}




