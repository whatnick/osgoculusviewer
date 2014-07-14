#include <osg/AnimationPath>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/XmlParser>
#include <osg/TexGen>

#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/FlightManipulator>

#include <osgSim/HeightAboveTerrain>

#include <osgParticle/PrecipitationEffect>
#include <osgParticle/SmokeEffect>
#include <osg/ShapeDrawable>

#include <stdio.h>

//Include windows specific header to find file paths
#include <Windows.h>

#include "SkyBox.h"
#include "AeroUtil.h"

struct Model : public osg::Referenced
{
    Model():
        position(0.0,0.0,0.0),
        scale(1.0,1.0,1.0),
        pivotPoint(0.0,0.0,0.0) {}

    osg::Vec3d                          position;
    osg::Vec3d                          scale;
    osg::Quat                           rotation;
    osg::Vec3d                          pivotPoint;
    osg::ref_ptr<osg::AnimationPath>    path;
    osg::ref_ptr<osg::Node>             node;
	std::string							model_name;
	std::string							extension;
};

class ReaderWriterScene : public osgDB::ReaderWriter
{
    public:

        ReaderWriterScene()
        {
            supportsExtension("scene","OpenSceneGraph Scene format");
        }

        virtual const char* className() const { return "OSG Scene Reader"; }

        virtual ReadResult readNode(const std::string& file, const osgDB::Options* opt) const
        {
            std::string ext = osgDB::getLowerCaseFileExtension(file);
            if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

            std::string fileName = osgDB::findDataFile( file, opt );
            if (fileName.empty()) return ReadResult::FILE_NOT_FOUND;

            // code for setting up the database path so that internally referenced file are searched for on relative paths.
            osg::ref_ptr<Options> local_opt = opt ? static_cast<Options*>(opt->clone(osg::CopyOp::SHALLOW_COPY)) : new Options;
            local_opt->getDatabasePathList().push_front(osgDB::getFilePath(fileName));

            osgDB::ifstream fin(fileName.c_str());
            if (fin)
            {
                return readNode(fin, local_opt.get());
            }

			return ReadResult::FILE_NOT_FOUND;
        }

        virtual ReadResult readNode(std::istream& fin, const osgDB::Options* options) const;

		virtual WriteResult writeNode(const osg::Node& node,const std::string& file,const osgDB::Options* opt) const
		{
			std::string ext = osgDB::getLowerCaseFileExtension(file);
            if (!acceptsExtension(ext)) return WriteResult::FILE_NOT_HANDLED;

            std::string fileName = file;

            // code for setting up the database path so that internally referenced file are searched for on relative paths.
            osg::ref_ptr<Options> local_opt = opt ? static_cast<Options*>(opt->clone(osg::CopyOp::SHALLOW_COPY)) : new Options;
            local_opt->getDatabasePathList().push_front(osgDB::getFilePath(fileName));

            osgDB::ofstream fout(fileName.c_str());
            if (fout)
            {
                return writeNode(node,fout, local_opt.get());
            }

			return WriteResult::ERROR_IN_WRITING_FILE;
		}

		virtual WriteResult writeNode(const osg::Node& node,std::ostream& fout,const osgDB::Options* opt) const;
};

osgDB::ReaderWriter::ReadResult ReaderWriterScene::readNode(std::istream& fin, const osgDB::Options* options) const
{
    osgDB::XmlNode::Input input;
    input.attach(fin);
    input.readAllDataIntoBuffer();

    osg::ref_ptr<osgDB::XmlNode> doc = new osgDB::XmlNode;
    doc->read(input);

    osgDB::XmlNode* root = 0;
    for(osgDB::XmlNode::Children::iterator itr = doc->children.begin();
        itr != doc->children.end() && !root;
        ++itr)
    {
        if ((*itr)->name=="scene") root = itr->get();
    }

    if (!root) return 0;

    std::string xPositionProperty("x");
    std::string yPositionProperty("y");
    std::string zPositionProperty("z");
    std::string scaleProperty("scale");
    std::string xRotationProperty("rx");
    std::string yRotationProperty("ry");
    std::string zRotationProperty("rz");

    osg::ref_ptr<osg::Node> terrain;
    osg::ref_ptr<osgParticle::PrecipitationEffect> precipitationEffect;
	osg::ref_ptr<osgParticle::SmokeEffect> smokeEffect;
	osg::ref_ptr<SkyBox> skyBox;
    osg::ref_ptr<osg::ClearNode> clearNode;

    typedef std::vector< osg::ref_ptr<Model> > Models;
	typedef std::set< std::string > Model_names;
    Models models;
	Model_names names;
	int dup_num = 0;
    for(osgDB::XmlNode::Children::iterator itr = root->children.begin();
    itr != root->children.end();
    ++itr)
    {
		if((*itr)->name=="smoke")
		{
			osgDB::XmlNode* smokeTag = itr->get();
			smokeEffect = new osgParticle::SmokeEffect;

			double smoke_x,smoke_y,smoke_z;
			if (smokeTag->properties.count(xPositionProperty)!=0)
            {
                smoke_x = osg::asciiToDouble(smokeTag->properties[xPositionProperty].c_str());
            }
            if (smokeTag->properties.count(yPositionProperty)!=0)
            {
               smoke_y = osg::asciiToDouble(smokeTag->properties[yPositionProperty].c_str());
            }
            if (smokeTag->properties.count(zPositionProperty)!=0)
            {
                smoke_z = osg::asciiToDouble(smokeTag->properties[zPositionProperty].c_str());
            }
			smokeEffect->setIntensity(0.4);
			smokeEffect->setPosition(osg::Vec3d(smoke_x,smoke_y,smoke_z));
			
		}
        if ((*itr)->name=="snow")
        {
            precipitationEffect = new osgParticle::PrecipitationEffect;
            double intensity = osg::asciiToDouble((*itr)->contents.c_str());
            precipitationEffect->snow(intensity);
        }
        if ((*itr)->name=="rain")
        {
            precipitationEffect = new osgParticle::PrecipitationEffect;
            double intensity = osg::asciiToDouble((*itr)->contents.c_str());
            precipitationEffect->rain(intensity);
        }
		if ((*itr)->name=="skybox_hills")
        {
            skyBox = new SkyBox;
			skyBox->getOrCreateStateSet()->setTextureAttributeAndModes( 0, new osg::TexGen );

			skyBox->setEnvironmentMap( 0,
				osgDB::readImageFile("Skybox_Hills/negx.jpg"), osgDB::readImageFile("Skybox_Hills/posx.jpg"),
				osgDB::readImageFile("Skybox_Hills/negy.jpg"), osgDB::readImageFile("Skybox_Hills/posy.jpg"),
				osgDB::readImageFile("Skybox_Hills/posz.jpg"), osgDB::readImageFile("Skybox_Hills/negz.jpg") );
        }
		if ((*itr)->name=="skybox_city")
        {
            skyBox = new SkyBox;
			skyBox->getOrCreateStateSet()->setTextureAttributeAndModes( 0, new osg::TexGen );

			skyBox->setEnvironmentMap( 0,
				osgDB::readImageFile("Skybox_City/negx.jpg"), osgDB::readImageFile("Skybox_City/posx.jpg"),
				osgDB::readImageFile("Skybox_City/negy.jpg"), osgDB::readImageFile("Skybox_City/posy.jpg"),
				osgDB::readImageFile("Skybox_City/posz.jpg"), osgDB::readImageFile("Skybox_City/negz.jpg") );
        }
        else if ((*itr)->name=="terrain")
        {
            osgDB::XmlNode* terrainTag = itr->get();
            std::string terrainFilename = terrainTag->contents;
            osg::ref_ptr<osg::Node> loadedNode = osgDB::readNodeFile(terrainFilename, options);
            if (loadedNode.valid())
            {
                terrain = loadedNode;
            }
        }
        else if ((*itr)->name=="model")
        {
            osgDB::XmlNode* modelTag = itr->get();
            std::string modelFilename = modelTag->contents;
            osg::ref_ptr<osg::Node> loadedNode = osgDB::readNodeFile(modelFilename, options);
            if (loadedNode.valid())
            {
                osg::ref_ptr<Model> model = new Model;
                model->node = loadedNode;
                if (modelTag->properties.count(xPositionProperty)!=0)
                {
                    model->position.x() = osg::asciiToDouble(modelTag->properties[xPositionProperty].c_str());
                }
                if (modelTag->properties.count(yPositionProperty)!=0)
                {
                    model->position.y() = osg::asciiToDouble(modelTag->properties[yPositionProperty].c_str());
                }
                if (modelTag->properties.count(zPositionProperty)!=0)
                {
                    model->position.z() = osg::asciiToDouble(modelTag->properties[zPositionProperty].c_str());
                }
                if (modelTag->properties.count(scaleProperty)!=0)
                {
                    model->scale.x() = model->scale.y() = model->scale.z() = osg::asciiToDouble(modelTag->properties[scaleProperty].c_str());
                }
                if (modelTag->properties.count(xRotationProperty)!=0)
                {
                    double angle = osg::DegreesToRadians(osg::asciiToDouble(modelTag->properties[xRotationProperty].c_str()));
                    model->rotation *= osg::Quat(angle, osg::Vec3d(1.0,0.0,0.0));
                }
                if (modelTag->properties.count(yRotationProperty)!=0)
                {
                    double angle = osg::DegreesToRadians(osg::asciiToDouble(modelTag->properties[yRotationProperty].c_str()));
                    model->rotation *= osg::Quat(angle, osg::Vec3d(0.0,1.0,0.0));
                }
                if (modelTag->properties.count(zRotationProperty)!=0)
                {
                    double angle = osg::DegreesToRadians(osg::asciiToDouble(modelTag->properties[zRotationProperty].c_str()));
                    model->rotation *= osg::Quat(angle, osg::Vec3d(0.0,0.0,1.0));
                }
				
				std::vector<std::string> bits = splitPath(modelFilename);

				std::string dir = bits[0];
				std::string fname = bits[1];
				std::string ext = bits[2];

				char tmp[256];

				std::string proposed_name = fname+ext;
				if(names.find(proposed_name)!=names.end())
				{
					dup_num++;
					std::sprintf(tmp, "%d", dup_num);
					proposed_name = proposed_name + "(" + tmp + ")";
				}
				else
				{
					dup_num = 0;
				}
				model->model_name = proposed_name;
				model->extension = ext;
				names.insert(proposed_name);
                osg::ComputeBoundsVisitor cbv;
                loadedNode->accept(cbv);
                const osg::BoundingBox& bb = cbv.getBoundingBox();
                model->pivotPoint.set((bb.xMin()+bb.xMax())*0.5, (bb.yMin()+bb.yMax())*0.5, bb.zMin());

                models.push_back(model);
            }
            else
            {
                osg::notify(osg::NOTICE)<<"Unable to load file:"<< modelFilename<<std::endl;
            }
        }
    }

    if (models.empty() && !terrain)
    {
        osg::notify(osg::NOTICE)<<"No terrain or models in scene file"<<std::endl;
        return 0;
    }

    osg::ref_ptr<osg::Group> group = new osg::Group;
    if (terrain.valid())
    {
		terrain.get()->setName("Terrain");
        group->addChild(terrain.get());

        double height = terrain->getBound().center().z() + terrain->getBound().radius();

        if (!models.empty())
        {
            osgSim::HeightAboveTerrain hat;
            for(unsigned int i=0; i<models.size();++i)
            {
                Model* model = models[i].get();
                hat.addPoint(osg::Vec3d(model->position.x(), model->position.z(), height));
            }

            hat.computeIntersections(terrain.get());

            for(unsigned int i=0; i<models.size();++i)
            {
                Model* model = models[i].get();

                osg::notify(osg::NOTICE)<<"model->position="<<model->position<<" HAT="<<hat.getHeightAboveTerrain(i)<<std::endl;
                model->position.z() = (hat.getPoint(i).z() - hat.getHeightAboveTerrain(i));
				
                osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
                pat->setPosition(model->position);

				if(!model->rotation.zeroRotation())
				{
					pat->setPivotPoint(model->pivotPoint);
					pat->setAttitude(model->rotation);
				}

                pat->setScale(model->scale);
                pat->addChild(model->node.get());
				pat->setName(model->model_name);
				pat->getOrCreateUserDataContainer();
				//pat->setUserValue("extension",model->extension);
                group->addChild(pat.get());
            }
        }
    }
	else
	{
		for(unsigned int i=0; i<models.size();++i)
        {
            Model* model = models[i].get();

            osg::notify(osg::NOTICE)<<"model->position="<<model->position<<std::endl;
			
            osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
            pat->setPosition(model->position);
			//If there is no rotation don't set rotation or pivot
			if(!model->rotation.zeroRotation())
			{
				pat->setPivotPoint(model->pivotPoint);
				pat->setAttitude(model->rotation);
			}
			pat->setScale(model->scale);
            pat->addChild(model->node.get());
			pat->setName(model->model_name);
			//pat->setUserValue("extension",model->extension);
			group->addChild(pat.get());
        }
	}

    if (precipitationEffect.valid())
    {
		precipitationEffect.get()->setName("Precipitation");
        group->getOrCreateStateSet()->setAttributeAndModes(precipitationEffect->getFog());
        group->addChild(precipitationEffect.get());

        clearNode = new osg::ClearNode;
        clearNode->setClearColor(precipitationEffect->getFog()->getColor());
		clearNode->setName("Background");
    }

    if (clearNode.valid())
    {
        group->addChild(clearNode.get());
    }

	if(smokeEffect.valid())
	{
		group->addChild(smokeEffect);
	}
	if (skyBox.valid())
	{
		//static int ReceivesShadowTraversalMask = 0x1;
		//static int CastsShadowTraversalMask = 0x2;
		skyBox.get()->setName("SkyBox");
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable( new osg::ShapeDrawable(
			new osg::Sphere(osg::Vec3(), group->getBound().radius()/20.0)) );
		geode->setCullingActive( false );
		skyBox->addChild( geode.get() );
		//skyBox->setNodeMask(skyBox->getNodeMask() & ~CastsShadowTraversalMask);;
		group->addChild(skyBox.get());
	}

    return group.release();
}

osgDB::ReaderWriter::WriteResult ReaderWriterScene::writeNode(const osg::Node& node,std::ostream& fout, const osgDB::Options* options) const
{
	const osg::Group* group = node.asGroup();
	unsigned int num_ch = group->getNumChildren();
	fout<<"<scene>"<<std::endl;
	for(unsigned int i=0;i<num_ch;i++)
	{
		const osg::Node* ch_node = group->getChild(i);
		if(std::strcmp(ch_node->className(),"PositionAttitudeTransform")==0)
		{
			const osg::PositionAttitudeTransform* pat = static_cast<const osg::PositionAttitudeTransform*>(ch_node);
			float x = pat->getPosition().x();
			float y = pat->getPosition().y();
			float z = pat->getPosition().z();
			fout<<"<model x=\""<<x<<"\" y=\""<<y<<"\" z=\""<<z<<"\" ";
			const osg::Quat attitude = pat->getAttitude();
			if(!attitude.zeroRotation())
			{
				double rx,ry,rz;
				getEulerFromQuat(pat->getAttitude(),rx,ry,rz);
				fout<<"rx=\""<<osg::RadiansToDegrees(rx)<<"\" ry=\""<<osg::RadiansToDegrees(ry)<<"\" rz=\""<<osg::RadiansToDegrees(rz)<<"\" ";
			}
			const osg::Vec3d scale = pat->getScale();
			if(scale.x()!=1.0)
			{
				fout<<" scale=\""<<scale.x()<<"\" ";
			}
			fout<<">";
			fout<<ch_node->getName()<<"</model>"<<std::endl;
		}
	}
	fout<<"</scene>"<<std::endl;
	fout.flush();
	
	return WriteResult::NOT_IMPLEMENTED;
}

// now register with Registry to instantiate the above
// reader/writer.
REGISTER_OSGPLUGIN(scene, ReaderWriterScene)
