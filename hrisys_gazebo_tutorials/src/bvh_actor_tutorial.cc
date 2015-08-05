#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"
#include "bvhactor.hh"

namespace gazebo
{
  class BVHActorTutorial : public WorldPlugin
  {
  public: BVHActorTutorial() : WorldPlugin()
    {
    }
    
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      // 	boost::bind(&BVHActorTutorial::OnUpdate, this));

      /// initialize sdf from sdf type
      std::string sdftype = _sdf->Get<std::string>("sdftype");
      if (sdf::findFile(sdftype).empty()) 
	sdftype = std::string(getenv("HRISYS_SDF_PATH")) + "/" + sdftype;
      sdf::SDFPtr modelSdf(new sdf::SDF);
      if (!sdf::initFile(sdftype, modelSdf))
	{
	  std::cerr << "bad sdf type" << "\n";
	  return;
	}

      /// overwrite or fill-in sdf with loaded file
      std::string instanceFile = common::find_file(_sdf->Get<std::string>("instance"));
      TiXmlDocument xmlDoc;
      if (instanceFile.empty())
	{
	  std::cerr << "instance file does not exist" << "\n";
	  return;
	}
      xmlDoc.LoadFile(instanceFile);
      TiXmlElement *elemXml = xmlDoc.FirstChildElement(modelSdf->root->GetName());
      /// sdf::readXml instead of sdf::readFile
      /// sdf::readFile requires a <sdf version="1.5"/> declaration in file,
      /// however this is unusual since the file is not an actual sdf
      if (!sdf::readXml(elemXml, modelSdf->root))
	{
	  std::cerr << "bad instance xml" << "\n";
	  return;
	}

      /// set a new model linked under world
      physics::BasePtr _parent = _world->GetByName(_world->GetName());
      this->model.reset(new physics::BVHactor(_parent));

      /// load model
      /// warning ->root depricated should be Root()
      this->model->Load(modelSdf->root); 
    }

  // public: void OnUpdate()
  //   {      
  //   };

  private: physics::BVHactorPtr model;

  // private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_WORLD_PLUGIN(BVHActorTutorial)
}
