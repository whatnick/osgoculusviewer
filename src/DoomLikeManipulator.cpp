/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

/* Written by Don Burns */

#include "DoomLikeManipulator.h"
#include <osgUtil/LineSegmentIntersector>

#include <osg/io_utils>

#ifndef M_PI
# define M_PI       3.14159265358979323846  /* pi */
#endif

using namespace osgGA;

DoomLikeManipulator::DoomLikeManipulator():
            _t0(0.0),
            _shift(false),
            _ctrl(false)
{ 
    _minHeightAboveGround          = 2.0;
    _minDistanceInFront            = 5.0;

    _speedAccelerationFactor       = 10.0;
    _speedDecelerationFactor       = 0.9;

    _maxSpeed                      = 15;
    _speedEpsilon                  = 0.02;
    


    _direction.set( 0,1,0);
    _upVector.set( 0,0,1);
    _homeUp = _upVector;
    _x=0;
    _y=0;
    _stop();
}

DoomLikeManipulator::~DoomLikeManipulator()
{
}

bool DoomLikeManipulator::intersect(const osg::Vec3d& start, const osg::Vec3d& end, osg::Vec3d& intersection) const
{
    osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(start,end);

    osgUtil::IntersectionVisitor iv(lsi.get());
    iv.setTraversalMask(_intersectTraversalMask);
    
    _node->accept(iv);
    
    if (lsi->containsIntersections())
    {
        intersection = lsi->getIntersections().begin()->getWorldIntersectPoint();
        return true;
    }
    return false;
}

void DoomLikeManipulator::setNode( osg::Node *node )
{
    _node = node;

    if (getAutoComputeHomePosition()) 
        computeHomePosition();

    home(0.0);
}

const osg::Node* DoomLikeManipulator::getNode() const
{
    return _node.get();
}

osg::Node* DoomLikeManipulator::getNode()
{
    return _node.get();
}


const char* DoomLikeManipulator::className() const 
{ 
    return "DoomLike"; 
}

void DoomLikeManipulator::setByMatrix( const osg::Matrixd &mat ) 
{
    _inverseMatrix = mat;
    _matrix.invert( _inverseMatrix );

    _position.set( _inverseMatrix(3,0), _inverseMatrix(3,1), _inverseMatrix(3,2 ));
    osg::Matrix R(_inverseMatrix);
    R(3,0) = R(3,1) = R(3,2) = 0.0;
    _direction = osg::Vec3(0,0,-1) * R; // camera up is +Z, regardless of CoordinateFrame

    _stop();
}

void DoomLikeManipulator::setByInverseMatrix( const osg::Matrixd &invmat) 
{
    _matrix = invmat;
    _inverseMatrix.invert( _matrix );

    _position.set( _inverseMatrix(3,0), _inverseMatrix(3,1), _inverseMatrix(3,2 ));
    osg::Matrix R(_inverseMatrix);
    R(3,0) = R(3,1) = R(3,2) = 0.0;
    _direction = osg::Vec3(0,0,-1) * R; // camera up is +Z, regardless of CoordinateFrame

    _stop();
}

osg::Matrixd DoomLikeManipulator::getMatrix() const
{
    return (_matrix);
}

osg::Matrixd DoomLikeManipulator::getInverseMatrix() const 
{
    return (_inverseMatrix );
}

void DoomLikeManipulator::computeHomePosition()
{
    _direction=osg::Vec3(1,0,0)*(osg::Vec3(1,0,0)*_direction)+osg::Vec3(0,1,0)*(osg::Vec3(0,1,0)*_direction);
    _direction.normalize();
    _upVector=_homeUp;
    if( !_node.valid() )
        return;

    osg::BoundingSphere bs = _node->getBound();

    /*
       * Find the ground - Assumption: The ground is the hit of an intersection
       * from a line segment extending from above to below the database at its 
       * horizontal center, that intersects the database closest to zero. */

	if(osg::equivalent(bs.radius(),0.0f))
	{
		return;
	}

    osg::Vec3 A = bs.center() + (_upVector*(bs.radius()*2));
    osg::Vec3 B = bs.center() + (-_upVector*(bs.radius()*2));

    if( (B-A).length() == 0.0)
    {
        return;
    }

    // start with it high
    double ground = bs.radius() * 10.0;

    osg::Vec3d ip;
    if (intersect(A, B, ip))
    {
        double d = ip.length();
        if( d < ground )
            ground = d;
    }
    else
    {
        //osg::notify(osg::WARN)<<"DoomLikeManipulator : I can't find the ground!"<<std::endl;
        ground = 0.0;
    }


    osg::Vec3 p(bs.center() + _upVector*( ground + _minHeightAboveGround*1.25 ) );
    setHomePosition( p, p + _direction, _homeUp );
}

void DoomLikeManipulator::init(const GUIEventAdapter&, GUIActionAdapter&)
{
    //home(ea.getTime());

    _stop();
}

void DoomLikeManipulator::home(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) 
{
    home(ea.getTime());
    us.requestRedraw();
    us.requestContinuousUpdate(false);

}

void DoomLikeManipulator::home(double) 
{
    if (getAutoComputeHomePosition()) 
		computeHomePosition();

    _position = _homeEye;
    _direction=osg::Vec3(1,0,0)*(osg::Vec3(1,0,0)*_direction)+osg::Vec3(0,1,0)*(osg::Vec3(0,1,0)*_direction);
    _upVector=_homeUp;
    _direction.normalize();
    
    _inverseMatrix.makeLookAt( _position, _position+_direction, _homeUp );
    _matrix.invert( _inverseMatrix );


    _forwardSpeed = 0.0;
    _sideSpeed = 0.0;
    _upSpeed = 0.0;
}

bool DoomLikeManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter &aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::FRAME):
            _frame(ea,aa);
            return false;
        default:
            break;
    }

    if (ea.getHandled()) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYUP):
            _keyUp( ea, aa );
            return false;
            break;

        case(osgGA::GUIEventAdapter::KEYDOWN):
            _keyDown(ea, aa);
            return false;
            break;
            
        case(osgGA::GUIEventAdapter::PUSH):
            _x = ea.getXnormalized();
            _y = ea.getYnormalized();
			if(ea.getButton()==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
			{
				_stop();
			}
            return false;
            break;
            
        case(osgGA::GUIEventAdapter::DRAG):
            aa.requestContinuousUpdate(true);
            if(_move(ea, aa))
                aa.requestRedraw();
            return false;
            break;

        case(osgGA::GUIEventAdapter::FRAME):
            _frame(ea,aa);
            return false;
            break;

        default:
            return false;
    }
}

void DoomLikeManipulator::getUsage(osg::ApplicationUsage& usage) const
{

    usage.addKeyboardMouseBinding("DoomLike : <SpaceBar>",        "Reset the view to the home position.");
    usage.addKeyboardMouseBinding("DoomLike : <Shift/SpaceBar>",  "Reset the up vector to the vertical.");
    usage.addKeyboardMouseBinding("DoomLike : <UpArrow>",         "Run forward.");
    usage.addKeyboardMouseBinding("DoomLike : <DownArrow>",       "Run backward.");
    usage.addKeyboardMouseBinding("DoomLike : <LeftArrow>",       "Step to the left.");
    usage.addKeyboardMouseBinding("DoomLike : <RightArrow>",      "Step to the right.");
    usage.addKeyboardMouseBinding("DoomLike : <Shift/UpArrow>",   "Move up.");
    usage.addKeyboardMouseBinding("DoomLike : <Shift/DownArrow>", "Move down.");
    usage.addKeyboardMouseBinding("DoomLike : <DragMouse>",       "Rotate the moving and looking direction.");
}



void DoomLikeManipulator::_keyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter & )
{
    switch( ea.getKey() )
    {
        case osgGA::GUIEventAdapter::KEY_Up:
        case osgGA::GUIEventAdapter::KEY_Down:
            if(!_shift)
                _decelerateForwardRate = true;
            else 
                _decelerateUpRate = true;
            break;

        case osgGA::GUIEventAdapter::KEY_Shift_L:
        case osgGA::GUIEventAdapter::KEY_Shift_R:
            _shift = false;
            _decelerateUpRate = true;
            break;

            
        case osgGA::GUIEventAdapter::KEY_Left:
        case osgGA::GUIEventAdapter::KEY_Right:
            if(!_shift)
                _decelerateSideRate = true;
            break;
    }
}

void DoomLikeManipulator::_keyDown( const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter & )
{
    switch( ea.getKey() )
    {
        case osgGA::GUIEventAdapter::KEY_Shift_L :
        case osgGA::GUIEventAdapter::KEY_Shift_R :
            _shift = true;
            break;

        case osgGA::GUIEventAdapter::KEY_Up:
            if( _shift )
            {
                _decelerateForwardRate = true;
                if(_upSpeed<_maxSpeed)
                    _upSpeed += _speedAccelerationFactor;
                _decelerateUpRate = false;
            }
            else
            {
                
                if(_forwardSpeed<_maxSpeed*2.f)
                    _forwardSpeed += _speedAccelerationFactor;
                _decelerateForwardRate = false;
            }
            break;

        case osgGA::GUIEventAdapter::KEY_Down:
            if( _shift )
            {
                _decelerateForwardRate = true;
                if(_upSpeed>-_maxSpeed)
                    _upSpeed -= _speedAccelerationFactor;
                _decelerateUpRate = false;
            }
            else
            {
                
                if(_forwardSpeed>-_maxSpeed*2.f)
                    _forwardSpeed -= _speedAccelerationFactor;
                _decelerateForwardRate = false;
            }
            break;

        case osgGA::GUIEventAdapter::KEY_Right:
            if( _shift )
            {
                _decelerateSideRate = true;
            }
            else
            {
                if(_sideSpeed<_maxSpeed)
                    _sideSpeed += _speedAccelerationFactor;
                _decelerateSideRate = false;
            }
            break;

        case osgGA::GUIEventAdapter::KEY_Left:
            if( _shift )
            {
                _decelerateSideRate = true;
            }
            else
            {
                if(_sideSpeed>-_maxSpeed)
                    _sideSpeed -= _speedAccelerationFactor;
                _decelerateSideRate = false;
            }
            break;
            


        case ' ':
            if(_shift)
            {
                osg::Vec3d position=_position;
                home(ea.getTime());
                _position=position;
            }
            else
            {
                home(ea.getTime());
            }
            break;
    }

}


bool DoomLikeManipulator::_move( const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter & )
{
    float dx = (ea.getXnormalized()-_x)*100.f;
    float dy = (ea.getYnormalized()-_y)*40.f; // less sensitivity
    _x = ea.getXnormalized();
    _y = ea.getYnormalized();

        
    osg::Vec3d up = _upVector;
    osg::Vec3d sv = _direction;
    osg::Vec3d lv = sv^up;
    lv.normalize();

    double yaw = osg::inDegrees(dx*10.0f*_dt);
    double pitch = osg::inDegrees(dy*50.0f*_dt);

    osg::Quat delta_rotate;

    osg::Quat pitch_rotate;
    osg::Quat yaw_rotate;

    yaw_rotate.makeRotate(yaw,0,0,1);
    pitch_rotate.makeRotate(pitch,lv.x(),lv.y(),lv.z());

    delta_rotate = yaw_rotate*pitch_rotate;

    // update the direction
    osg::Matrixd rotation_matrix;
    rotation_matrix.makeRotate(delta_rotate);
    _upVector = _upVector * rotation_matrix;
    _direction = _direction * rotation_matrix;
    return true;
    
}


void DoomLikeManipulator::_frame( const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter & )
{
    double t1 = ea.getTime();
    if( _t0 == 0.0 )
    {
        _t0 = ea.getTime();
        _dt = 0.0;
    }
    else
    {
        _dt = t1 - _t0;
        _t0 = t1;
    }

      osg::Vec3d _sideVec = _direction * osg::Matrix::rotate( -M_PI*0.5, _upVector);

      _position += ( (_direction       * _forwardSpeed) + 
                     (_sideVec         * _sideSpeed) +
                     (_upVector * _upSpeed) )* _dt;



    _adjustPosition();

    _inverseMatrix.makeLookAt( _position, _position + _direction, _upVector); 
    _matrix.invert(_inverseMatrix);

    if( _decelerateUpRate )
    {
        _upSpeed   *= _speedDecelerationFactor;
    }
    if( _decelerateSideRate )
    {
        _sideSpeed   *= _speedDecelerationFactor;
    }
    if( _decelerateForwardRate )
    {
        _forwardSpeed   *= _speedDecelerationFactor;
    } 
}

void DoomLikeManipulator::_adjustPosition()
{
    if( !_node.valid() )
        return;

    // Forward line segment at 3 times our intersect distance


    typedef std::vector<osg::Vec3d> Intersections;
    Intersections intersections;

    // Check intersects infront.
    osg::Vec3d ip;
    if (intersect(_position, 
                  _position + (_direction * (_minDistanceInFront * 3.0)),
                  ip ))
    {
        double d = (ip - _position).length();

        if( d < _minDistanceInFront )
        {
            _position = ip + (_direction * -_minDistanceInFront);
            _stop();
        }
    }
    
    // Check intersects below.

    if (intersect(_position, 
                  _position - _upVector*_minHeightAboveGround*3, 
                  ip ))
    {
        double d = (ip - _position).length();

        if( d < _minHeightAboveGround )
          _position = ip + (_upVector * _minHeightAboveGround);
    }
}


void DoomLikeManipulator::_stop()
{
    _forwardSpeed = 0.0;
    _sideSpeed = 0.0;
    _upSpeed = 0.0;
}

void DoomLikeManipulator::getCurrentPositionAsLookAt( osg::Vec3 &eye, osg::Vec3 &center, osg::Vec3 &up )
{
    eye = _position;
    center = _position + _direction;
    up.set(getUpVector(getCoordinateFrame(_position)));
}

