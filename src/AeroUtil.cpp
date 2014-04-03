#include <osg/Quat>
#include <osg/Node>
#include <osg/Group>

#include "AeroUtil.h"


std::vector<std::string> splitPath(std::string fullPath)
{
	char fname[255];
	char dir[255];
	char ext[20];
				
	#ifdef WIN32
	char drive[4];
	_splitpath(fullPath.c_str(),drive,dir,fname,ext);
	#endif


	#if defined(APPLE) || defined(UNIX)
	fname = BaseName(fullPath);
	#endif

	std::vector<std::string> out;
	out.push_back(std::string(drive)+std::string(dir));
	out.push_back(fname);
	out.push_back(ext);

	return out;
}

osg::Node* findNamedNode(const std::string& searchName, 
                                          osg::Node* currNode)
    {
       osg::Group* currGroup;
       osg::Node* foundNode;

       // check to see if we have a valid (non-NULL) node.
       // if we do have a null node, return NULL.
       if ( !currNode)
       {
          return NULL;
       }

       // We have a valid node, check to see if this is the node we 
       // are looking for. If so, return the current node.
       if (currNode->getName() == searchName)
       {
          return currNode;
       }

       // We have a valid node, but not the one we are looking for.
       // Check to see if it has children (non-leaf node). If the node
       // has children, check each of the child nodes by recursive call.
       // If one of the recursive calls returns a non-null value we have
       // found the correct node, so return this node.
       // If we check all of the children and have not found the node,
       // return NULL
       currGroup = currNode->asGroup(); // returns NULL if not a group.
       if ( currGroup ) 
       {
          for (unsigned int i = 0 ; i < currGroup->getNumChildren(); i ++)
          { 
             foundNode = findNamedNode(searchName, currGroup->getChild(i));
             if (foundNode)
                return foundNode; // found a match!
          }
          return NULL; // We have checked each child node - no match found.
       }
       else 
       {
          return NULL; // leaf node, no match 
       }
    }


void getEulerFromQuat(osg::Quat q, double& heading, double& attitude, double& bank)

{
	double limit = 0.499999;

	double sqx = q.x()*q.x();   
	double sqy = q.y()*q.y();   
	double sqz = q.z()*q.z();

	double t = q.x()*q.y() + q.z()*q.w();

	if (t>limit) // gimbal lock ?
	{
		heading = 2 * atan2(q.x(),q.w());
		attitude = osg::PI_2;
		bank = 0;
	}
	else if (t<-limit)
	{
		heading = -2 * atan2(q.x(),q.w());
		attitude = - osg::PI_2;
		bank = 0;
	}
	else
	{
		heading = atan2(2*q.y()*q.w()-2*q.x()*q.z() , 1 - 2*sqy - 2*sqz);
		attitude = asin(2*t);
		bank = atan2(2*q.x()*q.w()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqz);
	}
}

void getEulerFromMatrix( osg::Vec3& euler, const osg::Matrix& rotation )
{
   //implementation converted from plib's sg.cxx
   //PLIB - A Suite of Portable Game Libraries
   //Copyright (C) 1998,2002  Steve Baker
   //For further information visit http://plib.sourceforge.net

   osg::Matrix mat;

   osg::Vec3 col1(rotation(0, 0), rotation(0, 1), rotation(0, 2));
   double s = col1.length();

   const double magic_epsilon = 0.00001;
   if ( s <= magic_epsilon )
   {
      euler.set(0.0f, 0.0f, 0.0f);
      return ;
   }


   double oneOverS = 1.0f / s;
   for( int i = 0; i < 3; i++ )
      for( int j = 0; j < 3; j++ )
         mat(i, j) = rotation(i, j) * oneOverS;


   double sin_pitch = ClampUnity(mat(1, 2));
   double pitch = asin(sin_pitch);
   euler[1] = pitch;

   double cp = cos(pitch);

   if ( cp > -magic_epsilon && cp < magic_epsilon )
   {
      double cr = ClampUnity(-mat(2,1));
      double sr = ClampUnity(mat(0,1));

      euler[0] = 0.0f;
      euler[2] = atan2(sr,cr);
   }
   else
   {
      double one_over_cp = 1.0 / cp ;
      double sr = ClampUnity(-mat(0,2) * one_over_cp);
      double cr = ClampUnity(mat(2,2) * one_over_cp);
      double sh = ClampUnity(-mat(1,0) * one_over_cp);
      double ch = ClampUnity(mat(1,1) * one_over_cp);

      if ( ( osg::equivalent(sh,0.0,magic_epsilon) && osg::equivalent(ch,0.0,magic_epsilon) ) ||
           ( osg::equivalent(sr,0.0,magic_epsilon) && osg::equivalent(cr,0.0,magic_epsilon) ) )
      {
         cr = ClampUnity(-mat(2,1));
         sr = ClampUnity(mat(0,1));;

         euler[0] = 0.0f;
      }
      else
      {
        euler[0] = atan2(sh, ch);
      }

      euler[2] = atan2(sr, cr);
   }
}

float ClampUnity( float x )
{
   if ( x >  1.0f ) return  1.0f;
   if ( x < -1.0f ) return -1.0f;
   return x ;
}


// area3D_Polygon(): compute the area of a 3D planar polygon
//  Input:  int n = the number of vertices in the polygon
//          Point* V = an array of n+2 vertices in a plane with V[n]=V[0]
//          Point N = a normal vector of the polygon's plane
//  Return: the (float) area of the polygon
float
	area3D_Polygon( int n, const std::vector<osg::Vec3d> V_orig, osg::Vec3 N)
{
    float area = 0;
    float an, ax, ay, az; // abs value of normal and its coords
    int  coord;           // coord to ignore: 1=x, 2=y, 3=z
    int  i, j, k;         // loop indices

    if (n < 3) return 0;  // a degenerate polygon

	std::vector<osg::Vec3d> V(V_orig);

	osg::Vec3d v_0 = V[0];
	V.push_back(v_0);

    // select largest abs coordinate to ignore for projection
    ax = (N.x()>0 ? N.x() : -N.x());    // abs x-coord
    ay = (N.y()>0 ? N.y() : -N.y());    // abs y-coord
    az = (N.z()>0 ? N.z() : -N.z());    // abs z-coord

    coord = 3;                    // ignore z-coord
    if (ax > ay) {
        if (ax > az) coord = 1;   // ignore x-coord
    }
    else if (ay > az) coord = 2;  // ignore y-coord

    // compute area of the 2D projection
    for (i=1, j=2, k=0; i<n; i++, j++, k++) {
        switch (coord) {
          case 1:
            area += (V[i].y() * (V[j].z() - V[k].z()));
            continue;
          case 2:
            area += (V[i].x() * (V[j].z() - V[k].z()));
            continue;
          case 3:
            area += (V[i].x() * (V[j].y() - V[k].y()));
            continue;
        }
    }
    switch (coord) {    // wrap-around term
      case 1:
        area += (V[n].y() * (V[1].z() - V[n-1].z()));
        break;
      case 2:
        area += (V[n].x() * (V[1].z() - V[n-1].z()));
        break;
      case 3:
        area += (V[n].x() * (V[1].y() - V[n-1].y()));
        break;
    }

    // scale to get area before projection
    an = sqrt( ax*ax + ay*ay + az*az); // length of normal vector
    switch (coord) {
      case 1:
        area *= (an / (2*ax));
        break;
      case 2:
        area *= (an / (2*ay));
        break;
      case 3:
        area *= (an / (2*az));
    }
    return fabs(area);
}
//===================================================================
