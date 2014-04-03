/*
* main.cpp
*
*  Created on: Jul 03, 2013
*      Author: Bjorn Blissing
*/

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/KeySwitchMatrixManipulator>

#include "oculusViewConfig.h"
#include "DoomLikeManipulator.h"
#include "AeroTerrainManipulator.h"
#include "AeroUtil.h"

int main( int argc, char** argv )
{
	// use an ArgumentParser object to manage the program arguments.
	osg::ArgumentParser arguments(&argc,argv);

	//Register program running path to DB
	osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();
	char prog_name[1024];
	GetModuleFileName(NULL,prog_name,1024);
	std::string prog_path = splitPath(prog_name)[0];

	std::cout<<prog_path<<std::endl;
	std::cout<<pathList.size()<<std::endl;
	pathList.push_back(prog_path);

	// read the scene from the list of file specified command line arguments.
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);

	// if not loaded assume no arguments passed in, try use default cow model instead.
	//NO NEED FOR COWS !!if (!loadedModel) loadedModel = osgDB::readNodeFile("cow.osgt");

	if(loadedModel)
	{
		//Turn off lighting
		osg::StateSet* stateset = loadedModel->getOrCreateStateSet();
		stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	}
	// Still no loaded model, then exit
	if (!loadedModel) return 0;

	// Create Oculus View Config
	osg::ref_ptr<OculusViewConfig> oculusViewConfig = new OculusViewConfig;
	// Create viewer
	osgViewer::Viewer viewer(arguments);
	// Apply view config
	viewer.apply(oculusViewConfig);

	// Add loaded model to viewer
	viewer.setSceneData(loadedModel);

	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

	osgGA::CameraManipulator* manip = new AeroTerrainManipulator();
	manip->setAutoComputeHomePosition(true);
	keyswitchManipulator->addMatrixManipulator( '1', "AeroTerrain",  manip);


	//Add path based manipulators
	std::string pathfile;
	double animationSpeed = 1.0;
	while(arguments.read("--speed",animationSpeed) ) {}
	char keyForAnimationPath = '2';
	while (arguments.read("-p",pathfile))
	{
		osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
		if (apm || !apm->valid())
		{
			apm->setTimeScale(animationSpeed);

			unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
			keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
			keyswitchManipulator->selectMatrixManipulator(num);
			++keyForAnimationPath;
		}
	}

	viewer.setCameraManipulator(keyswitchManipulator.get());

	// Start Viewer
	return viewer.run();
}
