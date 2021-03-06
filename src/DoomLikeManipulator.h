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

#ifndef OSGGA_DOOM_LIKE_MANIPULATOR_DEF
#define OSGGA_DOOM_LIKE_MANIPULATOR_DEF 1

#include <iostream>

#include <osgGA/CameraManipulator>
#include <osg/Node>
#include <osg/Matrix>

/**
  \class osgGA::DoomLikeManipulator
  \brief A DoomLike manipulator driven with keybindings.

  The DoomLikeManipulator is better suited for applications that employ architectural walk-throughs.
  The camera control is fone via keyboard arrows concerning the position and via mouse draging concerning the orientation.
  There are two modes : the horizontal and the free. In the free one the translation direction is exactly aligned with the view direction and thus can span the whole set of direction. In the other, the moving direction is constrained to remain horizontal. Note : horizontal mode is not implemented yet!!
  Unlike most of the other manipulators, the user controls directly the speed of the camera and not its acceleration.
  As a result, the user can achieve fast moves and yet change quickly the direction of the movement.

  The DoomLike Manipulator allows the following movements with the listed
  Key combinations:
    \param SpaceBar         Reset the view to the home position.
    \param Shift/SpaceBar   Reset the up vector to the vertical.
    \param UpArrow          Run forward.
    \param DownArrow        Run backward.
    \param LeftArrow        Step to the left.
    \param RightArrow       Step to the right.
    \param Shift/UpArrow    Move up.
    \param Shift/DownArrow  Move down.
    \param Shift/Enter      Switch between horizontal and free mode.
    \param DragMouse        Rotate the moving and looking direction.
*/

namespace osgGA {

class DoomLikeManipulator : public osgGA::CameraManipulator
{

    public:
        /** Default constructor */
        DoomLikeManipulator();

        /** return className
          \return returns constant "DoomLike"
          */
        virtual const char* className() const;

        /** Set the current position with a matrix 
          \param matrix  A viewpoint matrix.
         */
        virtual void setByMatrix( const osg::Matrixd &matrix ) ;

        /** Set the current position with the inverse matrix
          \param invmat The inverse of a viewpoint matrix
          */
        virtual void setByInverseMatrix( const osg::Matrixd &invmat);

        /** Get the current viewmatrix */
        virtual osg::Matrixd getMatrix() const;

        /** Get the current inverse view matrix */
        virtual osg::Matrixd getInverseMatrix() const ;

        /** Set the  subgraph this manipulator is driving the eye through.
            \param node     root of subgraph
         */
        virtual void setNode(osg::Node* node);

        /** Get the root node of the subgraph this manipulator is driving the eye through (const)*/
        virtual const osg::Node* getNode() const;

        /** Get the root node of the subgraph this manipulator is driving the eye through */
        virtual osg::Node* getNode();

        /** Computes the home position based on the extents and scale of the 
            scene graph rooted at node */
        virtual void computeHomePosition();

        /** Sets the viewpoint matrix to the home position */
        virtual void home(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&) ;
        void home(double);

        virtual void init(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter&);

        /** Handles incoming osgGA events */
        bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter &aa);

        /** Reports Usage parameters to the application */
        void getUsage(osg::ApplicationUsage& usage) const;

        /** Report the current position as LookAt vectors */
        void getCurrentPositionAsLookAt( osg::Vec3 &eye, osg::Vec3 &center, osg::Vec3 &up );


        void setMinHeight( double in_min_height ) { _minHeightAboveGround = in_min_height; }
        double getMinHeight() const { return _minHeightAboveGround; }

        void setMinDistance( double in_min_dist ) { _minDistanceInFront = in_min_dist; }
        double getMinDistance() const { return _minDistanceInFront; }

        void setForwardSpeed( double in_fs ) { _forwardSpeed = in_fs; }
        double getForwardSpeed() const { return _forwardSpeed; }

        void setSideSpeed( double in_ss ) { _sideSpeed = in_ss; }
        double getSideSpeed() const { return _sideSpeed; }



    protected:

        virtual ~DoomLikeManipulator();

        bool intersect(const osg::Vec3d& start, const osg::Vec3d& end, osg::Vec3d& intersection) const;

        osg::ref_ptr<osg::Node> _node;
        osg::Matrixd _matrix;
        osg::Matrixd _inverseMatrix;

        double    _minHeightAboveGround;
        double    _minDistanceInFront;

        double    _speedEpsilon;
        double    _maxSpeed;
        double    _forwardSpeed;
        double    _sideSpeed;
        double    _upSpeed;
        double    _speedAccelerationFactor;
        double    _speedDecelerationFactor;

        bool      _decelerateSideRate;
        bool      _decelerateForwardRate;
        bool      _decelerateUpRate;
        
        double    _t0;
        double    _dt;
        osg::Vec3d _direction;  
        osg::Vec3d _upVector;
        double _x;
        double _y;
        osg::Vec3d _position;


        bool _shift;
        bool _ctrl;

        void _stop();
        void _keyDown( const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &);
        void _keyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter &);
        bool _move( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter &);
        void _frame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter &);

        void _adjustPosition();
};

}

#endif
