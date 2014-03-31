#include "AeroTerrainManipulator.h"

#include <osgUtil/LineSegmentIntersector>
#include <osg/io_utils>
#include <iostream>

using namespace osg;
using namespace osgGA;
using namespace osgUtil;

bool AeroTerrainManipulator::performMovementMiddleMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	// pan model.
    double scale = -0.3f * _distance * getThrowScale( eventTimeDelta );

    Matrixd rotation_matrix;
    rotation_matrix.makeRotate(_rotation);


    // compute look vector.
    Vec3d sideVector = getSideVector(rotation_matrix);

    // CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    // Vec3d localUp = getUpVector(coordinateFrame);
    Vec3d localUp = _previousUp;

    Vec3d forwardVector =localUp^sideVector;
    sideVector = forwardVector^localUp;

    forwardVector.normalize();
    sideVector.normalize();

    Vec3d dv = forwardVector * (dy*scale) + sideVector * (dx*scale);

    _center += dv;

    // need to recompute the intersection point along the look vector.

    bool hitFound = false;

    if (_node.valid())
    {
        // now reorientate the coordinate frame to the frame coords.
        CoordinateFrame coordinateFrame =  getCoordinateFrame(_center);

        // need to reintersect with the terrain
        double distance = _node->getBound().radius()*0.25f;

        coordinateFrame = getCoordinateFrame(_center);
        Vec3d new_localUp = getUpVector(coordinateFrame);


        Quat pan_rotation;
        pan_rotation.makeRotate(localUp,new_localUp);

        if (!pan_rotation.zeroRotation())
        {
            _rotation = _rotation * pan_rotation;
            _previousUp = new_localUp;
            //OSG_NOTICE<<"Rotating from "<<localUp<<" to "<<new_localUp<<"  angle = "<<acos(localUp*new_localUp/(localUp.length()*new_localUp.length()))<<std::endl;

            //clampOrientation();
        }
        else
        {
            OSG_INFO<<"New up orientation nearly inline - no need to rotate"<<std::endl;
        }
    }

    return true;
}


bool AeroTerrainManipulator::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
	if(ea.getEventType() == GUIEventAdapter::DOUBLECLICK)
	{
		if( getAnimationTime() <= 0. )
        {
            // center by mouse intersection (no animation)
            setCenterByMousePointerIntersection( ea, us );
        }
        else
        {
            // start new animation only if there is no animation in progress
            if( !isAnimating() )
                startAnimationByMousePointerIntersection( ea, us );

        }

		// perform zoom
        zoomModel( -_wheelZoomFactor*4, true );
        return true;
	}
	return super::handle(ea,us);
}

void AeroTerrainManipulator::setTransformation( const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up )
{
    if (!_node) return;

    // compute rotation matrix
    Vec3d lv(center-eye);
    _distance = lv.length();
    _center = center;

    OSG_INFO << "In compute"<< std::endl;

    if (_node.valid())
    {
        bool hitFound = false;

        double distance = lv.length();
        double maxDistance = distance+2*(eye-_node->getBound().center()).length();
        Vec3d farPosition = eye+lv*(maxDistance/distance);
        Vec3d endPoint = center;
        for(int i=0;
            !hitFound && i<2;
            ++i, endPoint = farPosition)
        {
            // compute the intersection with the scene.

            Vec3d ip;
            if (intersect(eye, endPoint, ip))
            {
                _center = ip;
                _distance = (ip-eye).length();

                hitFound = true;
            }
        }
    }

    // note LookAt = inv(CF)*inv(RM)*inv(T) which is equivalent to:
    // inv(R) = CF*LookAt.

    Matrixd rotation_matrix = Matrixd::lookAt(eye,center,up);

    //_rotation = rotation_matrix.getRotate().inverse();

    CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    _previousUp = getUpVector(coordinateFrame);

    clampOrientation();
}