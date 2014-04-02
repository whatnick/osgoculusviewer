#ifndef AEROUTIL_H
#define AEROUTIL_H

osg::Node* findNamedNode(const std::string& searchName, osg::Node* currNode);
void getEulerFromQuat(osg::Quat q, double& heading, double& attitude, double& bank);
float area3D_Polygon( int n, const std::vector<osg::Vec3d> V, osg::Vec3 N);
void getEulerFromMatrix( osg::Vec3& euler, const osg::Matrix& rotation );
float ClampUnity( float x );

#endif