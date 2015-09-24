#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"
#include "bvhactor.hh"
#include "LimbXtentions.hh"
#include "xtends/XBvhLimb.hh"
#include "xtends/XTrackerLimb.hh"
#include "xtends/XWalkerLimb.hh"
#include "xtends/XLegKinematicsLimb.hh"
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
	  std::cerr << "bad sdf type\n";
      	  return;
      	}

      std::string instanceFile = common::find_file(_sdf->Get<std::string>("instance"));
      TiXmlDocument xmlDoc;
      if (instanceFile.empty())
      	{
	  std::cerr << "instance file does not exist\n";
      	  return;
      	}
      xmlDoc.LoadFile(instanceFile);
      TiXmlElement *elemXml = xmlDoc.FirstChildElement(modelSdf->root->GetName());
      if (!sdf::readXml(elemXml, modelSdf->root))
      	{
	  std::cerr << "bad instance xml\n";
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
      		}
#ifdef HRISYS_HAVE_OPENNI
	      else if (xtentionType == "nite")
		{
		  this->xtentions["nite"].reset(new physics::XTrackerLimb(_world));
		}
#endif
	      else if (xtentionType == "legKinematics")
		{
		  this->xtentions["legKinematics"]
		    .reset(new physics::XLegKinematicsLimb(_world));
#ifdef HRISYS_HAVE_OPENNI
		  /// set linkage between legKinematics and nite
		  if (xtendedSdf->HasAttribute("linkage"))
		    {
		      auto it = this->xtentions.find("nite");
		      if (it != this->xtentions.end())
			boost::static_pointer_cast<physics::XLegKinematicsLimb>
			  (this->xtentions["legKinematics"])
			  ->SetFromTracker(boost::static_pointer_cast
					   <physics::XTrackerLimb>(it->second));
		      else
			std::cerr << "Illegal legKinematics declared before nite.\n";
		    }
#endif
		}
	      else if (xtentionType == "walker")
		{
		  this->xtentions["walker"].reset(new physics::XWalkerLimb(_world));
		  if (xtendedSdf->HasAttribute("linkage"))
		    {
		      auto it = this->xtentions.find("legKinematics");
		      if (it != this->xtentions.end())
			boost::static_pointer_cast<physics::XWalkerLimb>
			  (this->xtentions["walker"])
			  ->SetFromLegKinematics(boost::static_pointer_cast
						 <physics::XLegKinematicsLimb>(it->second));
		      else
			std::cerr << "Illegal walker declared befor legKinematics.\n";
		    }
		}
	      else
		{
		  xtendedSdf = xtendedSdf->GetNextElement("xtend");
		  continue;
		}
	      this->xtentions[xtentionType]->InitLimbX(xtendedSdf);
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
	      if (xtentions.find(xtentionType) == xtentions.end())
		{
		  startSdf = startSdf->GetNextElement("start");
		  continue;
		}
	      this->xtentions[xtentionType]
		->StartLimbX(this->model, startSdf->Get<std::string>("limb"),
			     startSdf->Get<std::string>("arg"));
	      startSdf = startSdf->GetNextElement("start");
      	    }
      	}
    }
    
  private: physics::XtendedActorPtr model;
  private: std::map<std::string, physics::LimbXtentionsPtr> xtentions;
  };
  
  GZ_REGISTER_WORLD_PLUGIN(XtendedActorTutorial)
}
