#ifndef AEROTERRAINMANIPULATOR_H
#define AEROTERRAINMANIPULATOR_H

#include <osgGA/TerrainManipulator>

class AeroTerrainManipulator : public osgGA::TerrainManipulator
{
	typedef osgGA::TerrainManipulator super;
	
	public: 
		AeroTerrainManipulator(int flags = DEFAULT_SETTINGS) : super(flags) { setAnimationTime( 1.0 ); } ;
		virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
		virtual void setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up );

	protected: 
		virtual ~AeroTerrainManipulator() {} ;
		virtual bool performMovementMiddleMouseButton( const double eventTimeDelta, const double dx, const double dy );
};

#endif