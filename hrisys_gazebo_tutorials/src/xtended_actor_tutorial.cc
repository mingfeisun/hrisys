#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"
#include "bvhactor.hh"
#include "LimbXtentions.hh"
#include "xtends/XBvhLimb.hh"
#include "XtendedActor.hh"

namespace gazebo
{
  class XtendedActorTutorial : public WorldPlugin
  {
  public: XtendedActorTutorial() : WorldPlugin()
    {
    }
    
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      /// >>> almost the same as bvh_actor_tutorial
      std::string sdftype = _sdf->Get<std::string>("sdftype");
      if (sdf::findFile(sdftype).empty()) 
      	sdftype = std::string(getenv("HRISYS_SDF_PATH")) + "/" + sdftype;
      sdf::SDFPtr modelSdf(new sdf::SDF);
      if (!sdf::initFile(sdftype, modelSdf))
      	{
      	  std::cerr << "bad sdf type" << "\n";
      	  return;
      	}

      std::string instanceFile = common::find_file(_sdf->Get<std::string>("instance"));
      TiXmlDocument xmlDoc;
      if (instanceFile.empty())
      	{
      	  std::cerr << "instance file does not exist" << "\n";
      	  return;
      	}
      xmlDoc.LoadFile(instanceFile);
      TiXmlElement *elemXml = xmlDoc.FirstChildElement(modelSdf->root->GetName());
      if (!sdf::readXml(elemXml, modelSdf->root))
      	{
      	  std::cerr << "bad instance xml" << "\n";
      	  return;
      	}

      physics::BasePtr _parent = _world->GetByName(_world->GetName());
      this->model.reset(new physics::XtendedActor(_parent));

      this->model->Load(modelSdf->root);
      /// almost the same as bvh_actor_tutorial <<<

      /// set the limb extentions
      if (_sdf->HasElement("xtend"))
      	{
      	  sdf::ElementPtr xtendedSdf = _sdf->GetElement("xtend");
      	  while (xtendedSdf)
      	    {
      	      std::string xtentionType = xtendedSdf->Get<std::string>("type");
      	      if (xtentionType == "bvh")
      		{
      		  this->xtentions["bvh"].reset(new physics::XBvhLimb(_world));
      		  this->xtentions["bvh"]->InitLimbX(xtendedSdf);
      		}
      	      xtendedSdf = xtendedSdf->GetNextElement("xtend");
      	    }
      	}

      /// start the limb extensions
      if (_sdf->HasElement("start"))
      	{
      	  sdf::ElementPtr startSdf = _sdf->GetElement("start");
      	  while (startSdf)
      	    {
      	      std::string xtentionType = startSdf->Get<std::string>("type");
      	      if (xtentionType == "bvh")
      		this->xtentions["bvh"]
      		  ->StartLimbX(this->model,
      			       startSdf->Get<std::string>("limb").c_str(),
      			       startSdf->Get<std::string>("file").c_str(),
      			       startSdf->Get<bool>("loop"));
      	      startSdf = startSdf->GetNextElement("start");
      	    }
      	}
    }
    
  private: physics::XtendedActorPtr model;
  private: std::map<std::string, physics::LimbXtentionsPtr> xtentions;
  };
  
  GZ_REGISTER_WORLD_PLUGIN(XtendedActorTutorial)
}
