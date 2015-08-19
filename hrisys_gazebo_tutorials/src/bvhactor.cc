#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/util/OpenAL.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/ContactManager.hh"

#include "bvhactor.hh"

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    BVHactor::BVHactor(BasePtr _parent) : Actor(_parent)
    {
      this->doDebug = false;
    };

    //////////////////////////////////////////////////
    BVHactor::~BVHactor() {};

    //////////////////////////////////////////////////
    void BVHactor::Load(sdf::ElementPtr _sdf)
    {
      /// add required elements for loading actor
      if (!_sdf->HasElement("animation"))
	  _sdf->AddElement("animation");

      if (!_sdf->HasElement("script"))
	{
	  _sdf->AddElement("script");
	  /// auto_start set to false from instance file
	  /// the following seems to not work with some versions of sdf
	  // _sdf->GetElement("script")->GetElement("auto_start")->Set(false);
	}

      Actor::Load(_sdf);

      /// set a default bvh map
      /// default assumes that bvh nodes and dae nodes have same name and number 
      std::map<std::string, std::string> skelMapDefault;
      for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
	skelMapDefault[this->skeleton->GetNodeByHandle(i)->GetName()]
	  = this->skeleton->GetNodeByHandle(i)->GetName();
      this->skelNodesMap["__default__"] = skelMapDefault;

      /// get the bvh_maps
      if (_sdf->HasElement("bvh_map"))
	{
	  sdf::ElementPtr mapSdf = _sdf->GetElement("bvh_map");
	  while (mapSdf)
	    {
	      std::map<std::string, std::string> skelMap;
	      sdf::ElementPtr pairSdf = mapSdf->GetElement("pair");
	      while (pairSdf)
		{
		  skelMap[pairSdf->Get<std::string>("dae")]
		    = pairSdf->Get<std::string>("bvh");
		  pairSdf = pairSdf->GetNextElement("pair");
		}
	      this->skelNodesMap[mapSdf->Get<std::string>("name")] = skelMap;
	      mapSdf = mapSdf->GetNextElement("bvh_map");
	    }
	}

      /// add bvh animations
      if (_sdf->HasElement("bvh_animation"))
	{
	  sdf::ElementPtr bvhSdf = _sdf->GetElement("bvh_animation");
	  while (bvhSdf)
	    {
	      this->AddBVHAnimation(bvhSdf);
	      bvhSdf = bvhSdf->GetNextElement("bvh_animation");
	    }
	}

      /// show initial bvh pose if debug is true
      if (this->doDebug)
	{
	  this->SetPose(this->debugPose, this->world->GetSimTime().Double());
	}

      /// automatic playing of bvh
      if (_sdf->HasElement("auto_start"))
	{
	  if (_sdf->Get<std::string>("auto_start") != "__no_auto_start__")
	    this->StartBVH(_sdf->Get<std::string>("auto_start"));
	}

#ifdef HRISYS_HAVE_OPENAL
      if (_sdf->HasElement("add_audio_to_link"))
	{
	  sdf::ElementPtr addToLinkSdf = _sdf->GetElement("add_audio_to_link");

	  while (addToLinkSdf)
	    {
	      this->AddAudioToLink(addToLinkSdf);
	      addToLinkSdf = addToLinkSdf->GetNextElement("add_audio_to_link");
	    }
	}
#endif
    }

    //////////////////////////////////////////////////
    void BVHactor::Fini()
    {
#ifdef HRISYS_HAVE_OPENAL
      for (std::map<std::string, gazebo::util::LinkAudio>::iterator iter = this->linkAudio.begin();
	   iter != this->linkAudio.end(); ++iter)
	{
	  // this->world->GetPhysicsEngine()->GetContactManager()
	  //   ->RemoveFilter(this->GetScopedName() + "/audio_collision");
	  iter->second.audioSink.reset();
	}
#endif

      Actor::Fini();
    }

    //////////////////////////////////////////////////
    void BVHactor::StartBVH(std::string bvhfile)
    {
      std::map<std::string, common::SkeletonAnimation*>::iterator itr
	= this->skelAnimation.find(bvhfile);

      if (itr == this->skelAnimation.end())
      	{
      	  std::cerr << "BVH animation does not exist" << "\n";
      	  return;
      	}

      this->bvhfileOnPlay = bvhfile;
      this->scriptLength = this->skelAnimation[this->bvhfileOnPlay]->GetLength();
      this->active = true;
      this->playStartTime = this->world->GetSimTime();
      this->lastScriptTime = std::numeric_limits<double>::max();
    }

    //////////////////////////////////////////////////
    common::Skeleton* BVHactor::LoadBVH(const std::string &_filename,
					std::map<std::string, std::string> _skelMap,
					double _scale,
					bool _debug)
    {
      std::string fullname = common::find_file(_filename);

      common::Skeleton *skeleton = NULL;
      std::ifstream file;
      file.open(fullname.c_str());
      std::vector<common::SkeletonNode*> nodes;
      std::vector<std::vector<std::string> > nodeChannels;
      unsigned int totalChannels = 0;
      std::string line;

      /// get node structure of bvh
      if (file.is_open())
	{
	  getline(file, line);
	  if (line.find("HIERARCHY") == std::string::npos)
	    {
	      /// if node structure is not defined
	      file.close();
	      return NULL;
	    }

	  common::SkeletonNode *parent = NULL;
	  common::SkeletonNode *node = NULL;

	  /// load the nodes
	  while (!file.eof())
	    {
	      getline(file, line);
	      std::vector<std::string> words;
	      boost::trim(line);
	      boost::split(words, line, boost::is_any_of("   "));

	      if (words[0] == "ROOT" || words[0] == "JOINT")
		{
		  /// line defines a root or joint
		  if (words.size() < 2)
		    {
		      /// root or joint does not have a name
		      file.close();
		      return NULL;
		    }
		  common::SkeletonNode::SkeletonNodeType type = common::SkeletonNode::JOINT;
		  std::string name = words[1];
		  node = new common::SkeletonNode(parent, name, name, type);
		  nodes.push_back(node);
		}
	      else
		{
		  /// line describes a root or joint

		  if (words[0] == "OFFSET")
		    {
		      if (words.size() < 4)
		  	{
		  	  /// number of initial pose data not enough
		  	  file.close();
		  	  return NULL;
		  	}
		      math::Matrix4 transform(math::Matrix4::IDENTITY);
		      math::Vector3 bvhOffset = math::Vector3(math::parseFloat(words[1]),
							       math::parseFloat(words[2]),
							       math::parseFloat(words[3]));
		      transform.SetTranslate(bvhOffset);
		      node->SetTransform(transform, false);
		    }

		  else if (words[0] == "CHANNELS")
		    {
		      if (words.size() < 3 ||
			  static_cast<size_t>(math::parseInt(words[1]) + 2) >
			  words.size())
			{
			  /// number of channels not defined
			  /// or not enough channel description
			  file.close();
			  return NULL;
			}
		      nodeChannels.push_back(words);
		      totalChannels += math::parseInt(words[1]);
		    }

		  else if (words.size() == 2 && words[0] == "End"
			   && words[1] == "Site")
		    {
		      /// ignore End Sites
		      getline(file, line);  /// read {
		      getline(file, line);  /// read OFFSET
		      getline(file, line);  /// read }
		    }

		  else
		    {
		      if (words[0] == "{")
			{
			  /// next is child
			  parent = node;
			}

		      else if (words[0] == "}")
			{
			  /// next is parent
			  parent = parent->GetParent();
			}

		      else
			{
			  /// element is not a part of HIERARCHY

			  if (nodes.empty())
			    {
			      /// no nodes were defined in file
			      file.close();
			      return NULL;
			    }

			  /// expects MOTION which fits none of the ifs
			  /// should be a safer check; like check parent
			  skeleton = new common::Skeleton(nodes[0]);
			  break;

			  /// there should be a skip for any other elements
			}
		    }
		}

	    } /// while
	} /// file.is_open(); load nodes ends here


      double autoScale = 0.0;
      unsigned int zeroTranslationNodeCount = 0;

      /// check if sdf <map> defines correct joint pairs
      if (_skelMap.size() != nodes.size())
	{
	  /// fail : skel map does not define enough joints
	  std::cerr << "<map> has illegal number of joints! expected "
		    << nodes.size() << " but has " << _skelMap.size() << "\n";
	  return NULL;
	}
      else
	{
	  for (std::map<std::string, std::string>::iterator iter =
		 _skelMap.begin(); iter != _skelMap.end(); ++iter)
	    {
	      common::SkeletonNode *animNode;
	      for (unsigned int i = 0; i < nodes.size(); ++i)
		  if (nodes[i]->GetName() == iter->second)
		    animNode = nodes[i];
	      common::SkeletonNode *skinNode = this->skeleton->GetNodeByName(iter->first);

	      if (!animNode || !skinNode)
		{
		  /// fail : expected joint does not exist
		  std::cerr << "<map> has wrong joint name" << "\n";
		  return NULL;
		}

	      if (animNode->GetChildCount() != skinNode->GetChildCount())
		{
		  /// fail: child number differs, thus joints aren't comapatible
		  std::cerr << "<map> defines a non compatible joint pair"
			    << iter->first << " and " << iter->second << "\n";
		  return NULL;
		}
	      else
		{
		  /// setup nodes for aligning
		  /// convert bvh node names to dae node names
		  animNode->SetName(iter->first);
		  math::Matrix4 transform(math::Matrix4::IDENTITY);
		  math::Vector3 bvhOffset = animNode->GetTransform().GetTranslation();
		  math::Vector3 daeOffset
		    = this->skeleton->GetNodeByName(iter->first)
		    ->GetTransform().GetTranslation();
		  /// scale bvh offset to dae link length
		  transform.SetTranslate(daeOffset.GetLength() * bvhOffset.Normalize());
		  animNode->SetTransform(transform, false);
		  /// update autoScale
		  if (bvhOffset.GetLength() == 0.0)
		    zeroTranslationNodeCount += 1;
		  else
		    autoScale += static_cast<double>(daeOffset.GetLength()) / bvhOffset.GetLength();
		}
	    }
	}

      /// calculate scale for root translation
      if (nodes.size() == zeroTranslationNodeCount)
	{
	  std::cerr << "This is unusual bone structure. All bones have zero length!" << "\n";
	  autoScale = 1.0;
	}
      else
	{
	  autoScale = autoScale / (nodes.size() - zeroTranslationNodeCount);
	}

      if (_scale == 0.0)
	_scale = autoScale;

      /// calculate translationAligner : aligner of initial bvh pose to initial dae pose
      std::map<std::string, math::Matrix4> translationAligner;
      for (unsigned int i = 0; i < nodes.size(); i++)
      	{
      	  common::SkeletonNode *node = nodes[i];

      	  if (node->GetParent() != NULL)
      	    {
      	      if (node->GetParent()->GetChildCount() > 1)
      		{
      		  /// parent link has multiple child links
      		  if (this->skeleton->GetNodeByName(node->GetName())
      		      ->GetTransform().GetTranslation() == math::Vector3(0, 0, 0))
      		    {
      		      /// parent link is a virtual link (has no length)
		      translationAligner[node->GetName()] = math::Matrix4::IDENTITY;
      		    }
      		  else
      		    {
      		      /// ---------------------------------------------------------
      		      /// ------------------------ TODO ---------------------------
      		      /// ---------------------------------------------------------
      		    }
      		}
      	    }


      	  if (node->GetChildCount() == 0)
      	    /// if this is an end bone, then link i matrix is already calculated
      	    continue;


      	  if (node->GetName() == this->skeleton->GetRootNode()->GetName())
      	    {
      	      /// if this is root, then some setup is needed to match bvh and dae
      	      translationAligner[node->GetName()]
      		= this->skeleton->GetRootNode()->GetTransform().GetRotation().GetAsMatrix4();
	      
      	      math::Matrix4 tmp = translationAligner[node->GetName()];
      	      tmp.SetTranslate(node->GetTransform().GetTranslation());
      	      node->SetTransform(tmp, true);
      	    }


      	  if (node->GetChildCount() > 1)
      	      /// if link i has multiple childs
      	      continue;

      	  /// else, which means link i only has a single child link i+1

      	  /// get link i+1 posture direction in world coordinates
      	  math::Vector3 relativeBVH = node->GetChild(0)->GetModelTransform().GetTranslation()
      	    - node->GetModelTransform().GetTranslation();
      	  math::Vector3 relativeDAE
      	    = this->skeleton->GetNodeByName(node->GetName())->GetChild(0)
	    ->GetModelTransform().GetTranslation()
      	    - this->skeleton->GetNodeByName(node->GetName())->GetModelTransform().GetTranslation();

      	  if (relativeBVH == math::Vector3(0, 0, 0)
      	      || relativeDAE == math::Vector3(0, 0, 0))
      	    {
      	      /// unexpected
      	      std::cerr << "Duplicated joint found! This might cause some errors!" << std::endl;
      	      continue;
      	    }

      	  /// calculate world coordinate rotation quaternion (difference between link i+1 posture)
      	  math::Vector3 n = relativeBVH.Cross(relativeDAE);
      	  double theta = asin(n.GetLength() / (relativeDAE.GetLength() * relativeBVH.GetLength()));

      	  /// calculate bvh to dae of link i+1
      	  translationAligner[node->GetChild(0)->GetName()]
	    = this->skeleton->GetNodeByName(node->GetName())
	    ->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
      	    * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
      	    * node->GetModelTransform().GetRotation().GetAsMatrix4();

      	  /// fix bvh posture of all links until link i, so that bvh and dae world posture matches
      	  math::Matrix4 tmp = node->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
      	    * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
      	    * node->GetModelTransform().GetRotation().GetAsMatrix4();
      	  tmp.SetTranslate(node->GetTransform().GetTranslation());
      	  node->SetTransform(tmp, true);
      	}


      /// now, bvh and dae should have the same world posture
      /// calculate the rotationAligner : aligner of initial bvh pose to initial dae pose
      std::map<std::string, math::Matrix4> rotationAligner;
      for (unsigned int i = 0; i < nodes.size(); i++)
	{
      	  common::SkeletonNode *node = nodes[i];

      	  if (node->GetName() == this->skeleton->GetRootNode()->GetName())
      	    {
	      /// rotation should not be aligned with root
	      rotationAligner[node->GetName()] = math::Matrix4::IDENTITY;
	      continue;
	    }

	  /// in case an aligner was not correctly calculated, set a value to prevent nan
	  if (translationAligner[node->GetName()] == math::Matrix4::ZERO)
	    translationAligner[node->GetName()] = math::Matrix4::IDENTITY;

	  rotationAligner[node->GetName()]
	    = node->GetTransform().GetRotation().GetAsMatrix4().Inverse()
	    * translationAligner[node->GetName()].Inverse()
	    * this->skeleton->GetNodeByName(node->GetName())->GetTransform().GetRotation().GetAsMatrix4();
	}


      /// if sdf requests debug
      if (_debug)
	{
	  std::cout << "/////////////////////////////////////////" << std::endl;
	  std::cout << "Showing debug: " << _filename << std::endl << std::endl;

	  for (unsigned int i = 0; i < nodes.size(); i++)
	    {
	      common::SkeletonNode *node = nodes[i];
	      math::Matrix4 transform(math::Matrix4::IDENTITY);
	      transform.SetTranslate(node->GetTransform().GetTranslation());
	      transform
		= translationAligner[node->GetName()] * transform * rotationAligner[node->GetName()];
	      std::cout << node->GetName() << std::endl;
	      std::cout << "position align matrix" << std::endl;
	      std::cout << translationAligner[node->GetName()];
	      std::cout << "rotation align matrix" << std::endl;
	      std::cout << rotationAligner[node->GetName()];
	      std::cout << "dae" << std::endl;
	      std::cout << this->skeleton->GetNodeByName(node->GetName())->GetTransform();
	      std::cout << "bvh" << std::endl;
	      std::cout << transform << std::endl;	      
	      this->debugPose[node->GetName()] = transform;
	  }

	  this->doDebug = true;
	  std::cout << "/////////////////////////////////////////" << std::endl;
	}



      /// from here get animation      
      getline(file, line);
      std::vector<std::string> words;
      boost::trim(line);
      boost::split(words, line, boost::is_any_of("   "));
      unsigned int frameCount = 0;
      double frameTime = 0.0;

      if (words[0] != "Frames:" || words.size() < 2)
	{
	  /// no definition of motions or number of frames is not defined
	  file.close();
	  return NULL;
	}
      
      frameCount = math::parseInt(words[1]);

      getline(file, line);
      words.clear();
      boost::trim(line);
      boost::split(words, line, boost::is_any_of("   "));

      if (words.size() < 3 || words[0] != "Frame" || words[1] != "Time:")
	{
	  /// no definition of frame time
	  file.close();
	  return NULL;
	}

      frameTime = math::parseFloat(words[2]);

      /// time stores length of animation
      double time = 0.0;
      unsigned int frameNo = 0;
      common::SkeletonAnimation *animation = new common::SkeletonAnimation(_filename);

      /// load the motions
      while (!file.eof())
	{
	  getline(file, line);
	  words.clear();
	  boost::trim(line);
	  boost::split(words, line, boost::is_any_of("   "));

	  if (words.size() < totalChannels)
	    {
	      /// number of frame data does not match expected channels
	      gzwarn << "Frame " << frameNo << " invalid.\n";
	      frameNo++;
	      time += frameTime;
	      continue;
	    }

	  /// parse data of nth frame
	  /// cursor points to xth data value in line
	  unsigned int cursor = 0;
	  for (unsigned int i = 0; i < nodes.size(); i++)
	    {
	      common::SkeletonNode *node = nodes[i];

	      /// channels = {"CHANNELS", number_of_channels, channel_type1, ......}
	      std::vector<std::string> channels = nodeChannels[i];

	      math::Vector3 xAxis(1, 0, 0);
	      math::Vector3 yAxis(0, 1, 0);
	      math::Vector3 zAxis(0, 0, 1);
	      double xAngle = 0.0;
	      double yAngle = 0.0;
	      double zAngle = 0.0;

	      math::Matrix4 transform(math::Matrix4::IDENTITY);
	      math::Vector3 translation = node->GetTransform().GetTranslation();
	      std::vector<math::Matrix4> mats;

	      unsigned int chanCount = math::parseInt(channels[1]);
	      for (unsigned int j = 2; j < (2 + chanCount); j++)
		{
		  /// until 2 + chanCount, listed values are for node i
		  double value = math::parseFloat(words[cursor]);
		  cursor++;
		  std::string channel = channels[j];
		  if (channel == "Xposition")
		    translation.x = value * _scale;
		  else if (channel == "Yposition")
		    translation.y = value * _scale;
		  else if (channel == "Zposition")
		    translation.z = value * _scale;

		  if (channel == "Zrotation")
		    {
		      zAngle = GZ_DTOR(value);
		      mats.push_back(math::Quaternion(zAxis, zAngle).GetAsMatrix4());
		    }
		  else if (channel == "Xrotation")
		    {
		      xAngle = GZ_DTOR(value);
		      mats.push_back(math::Quaternion(xAxis, xAngle).GetAsMatrix4());
		    }
		  else if (channel == "Yrotation")
		    {
		      yAngle = GZ_DTOR(value);
		      mats.push_back(math::Quaternion(yAxis, yAngle).GetAsMatrix4());
		    }
		}

	      /// calculate transform (euler -> matrix)
	      while (!mats.empty())
		{
		  transform = mats.back() * transform;
		  mats.pop_back();
		}

	      transform.SetTranslate(translation);
	      transform
		= translationAligner[node->GetName()] * transform * rotationAligner[node->GetName()];

	      /// add this transformation matrix for node i for nth frame
	      animation->AddKeyFrame(node->GetName(), time, transform);
	    }

	  frameNo++;
	  time += frameTime;

	  if (frameNo == frameCount)
	    /// end of all frames
	    break;
	}

      if (frameNo < frameCount - 1)
	/// expected number of frames not found in file, this is not fatal
	gzwarn << "BVH file ended unexpectedly.\n";

      skeleton->AddAnimation(animation);

      file.close();
      return skeleton;
    }


    //////////////////////////////////////////////////
    void BVHactor::AddBVHAnimation(sdf::ElementPtr _sdf)
    {
      std::string animName = _sdf->Get<std::string>("name");

      std::string animFile = _sdf->Get<std::string>("filename");
      std::string extension = animFile.substr(animFile.rfind(".") + 1,
					      animFile.size());
      double animScale = _sdf->Get<double>("scale");
      std::string animMap = _sdf->Get<std::string>("map");
      bool animDebug = _sdf->Get<bool>("debug");

      common::Skeleton *skel = NULL;

      if (extension == "bvh")
	{
	  /// get the skeleton pair map
	  std::map<std::string, std::string> skelMap;

	  if (skelNodesMap.find(animMap) == skelNodesMap.end())
	    skelMap = skelNodesMap["__default__"];
	  else
	    skelMap = skelNodesMap[animMap];

	  skel = this->LoadBVH(animFile, skelMap, animScale, animDebug);
	}
      else
	{
	  return;
	}

      if (!skel || skel->GetNumAnimations() == 0)
	{
	  std::cerr << "Failed to load BVH animation : " << animName << "\n";
	  return;
	}


      this->skelAnimation[animName] = skel->GetAnimation(0);
      this->interpolateX[animName] = _sdf->Get<bool>("interpolate_x");
    }

    //////////////////////////////////////////////////
    void BVHactor::Update()
    {
      if (this->autoStart)
	{
	  Actor::Update();
	  return;
	}

      if (!this->active)
	return;

      common::Time currentTime = this->world->GetSimTime();

      /// do not refresh animation more faster the 30 Hz sim time
      /// TODO: Reducing to 20 Hz. Because there were memory corruption
      /// and segmentation faults. Possibly due to some dangling pointers
      /// in pose message processing. This will need a proper fix. Just a
      /// workaround for now.
      if ((currentTime - this->prevFrameTime).Double() < (1.0 / 20.0))
	return;

      double scriptTime = currentTime.Double() - this->startDelay -
	this->playStartTime.Double();

      /// waiting for delayed start
      if (scriptTime < 0)
	return;

      if (scriptTime >= this->scriptLength)
	{
	  if (!this->loop)
	    {
	      return;
	    }
	  else
	    {
	      scriptTime = scriptTime - this->scriptLength;
	      this->playStartTime = currentTime - scriptTime;
	    }
	}

      /// at this point we are certain that a new frame will be animated
      this->prevFrameTime = currentTime;

      common::SkeletonAnimation *skelAnim = this->skelAnimation[this->bvhfileOnPlay];
      std::map<std::string, math::Matrix4> frame;
      frame = skelAnim->GetPoseAt(scriptTime);

      math::Matrix4 rootTrans =
	frame[this->skeleton->GetRootNode()->GetName()];
      math::Vector3 rootPos = rootTrans.GetTranslation();
      math::Quaternion rootRot = rootTrans.GetRotation();
      math::Pose modelPose;
      /// modelPose = initial model position
      math::Pose actorPose;
      actorPose.pos = modelPose.pos + modelPose.rot.RotateVector(rootPos);
      actorPose.rot = modelPose.rot * rootRot;
      math::Matrix4 rootM(actorPose.rot.GetAsMatrix4());
      rootM.SetTranslate(actorPose.pos);
      frame[this->skeleton->GetRootNode()->GetName()] = rootM;

      this->SetPose(frame, currentTime.Double());
      this->lastScriptTime = scriptTime;

#ifdef HRISYS_HAVE_OPENAL
      this->UpdateAudio();
#endif
    }

    //////////////////////////////////////////////////
    void BVHactor::SetPose(std::map<std::string, math::Matrix4> _frame, double _time)
    {
      msgs::PoseAnimation msg;
      msg.set_model_name(this->visualName);
      msg.set_model_id(this->visualId);

      math::Matrix4 modelTrans(math::Matrix4::IDENTITY);
      math::Pose mainLinkPose;

      for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); i++)
	{
	  common::SkeletonNode *bone = this->skeleton->GetNodeByHandle(i);
	  common::SkeletonNode *parentBone = bone->GetParent();
	  math::Matrix4 transform(math::Matrix4::IDENTITY);
	  if (_frame.find(bone->GetName()) != _frame.end())
	    transform = _frame[bone->GetName()];
	  else
	    transform = bone->GetTransform();

	  LinkPtr currentLink = this->GetChildLink(bone->GetName());
	  math::Pose bonePose = transform.GetAsPose();

	  if (!bonePose.IsFinite())
	    {
	      std::cerr << "ACTOR: " << _time << " " << bone->GetName()
			<< " " << bonePose << "\n";
	      bonePose.Correct();
	    }

	  msgs::Pose *bone_pose = msg.add_pose();
	  bone_pose->set_name(bone->GetName());

	  if (!parentBone)
	    {
	      bone_pose->mutable_position()->CopyFrom(msgs::Convert(math::Vector3()));
	      bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(
								       math::Quaternion()));
	      mainLinkPose = bonePose;
	    }
	  else
	    {
	      bone_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.pos));
	      bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.rot));
	      LinkPtr parentLink = this->GetChildLink(parentBone->GetName());
	      math::Pose parentPose = parentLink->GetWorldPose();
	      math::Matrix4 parentTrans(parentPose.rot.GetAsMatrix4());
	      parentTrans.SetTranslate(parentPose.pos);
	      transform = parentTrans * transform;
	    }

	  msgs::Pose *link_pose = msg.add_pose();
	  link_pose->set_name(currentLink->GetScopedName());
	  link_pose->set_id(currentLink->GetId());
	  math::Pose linkPose = transform.GetAsPose() - mainLinkPose;
	  link_pose->mutable_position()->CopyFrom(msgs::Convert(linkPose.pos));
	  link_pose->mutable_orientation()->CopyFrom(msgs::Convert(linkPose.rot));
	  currentLink->SetWorldPose(transform.GetAsPose(), true, false);
	}

      msgs::Time *stamp = msg.add_time();
      stamp->CopyFrom(msgs::Convert(_time));

      msgs::Pose *model_pose = msg.add_pose();
      model_pose->set_name(this->GetScopedName());
      model_pose->set_id(this->GetId());
      model_pose->mutable_position()->CopyFrom(msgs::Convert(mainLinkPose.pos));
      model_pose->mutable_orientation()->CopyFrom(msgs::Convert(mainLinkPose.rot));

      if (this->bonePosePub && this->bonePosePub->HasConnections())
	this->bonePosePub->Publish(msg);
      this->SetWorldPose(mainLinkPose, true, false);
    }

#ifdef HRISYS_HAVE_OPENAL
    //////////////////////////////////////////////////
    void BVHactor::AddAudioToLink(sdf::ElementPtr _sdf)
    {
      std::string link = _sdf->Get<std::string>("link");

      bool findLink = false;
      for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
	if (link == this->skeleton->GetNodeByHandle(i)->GetName())
	  findLink = true;

      if (findLink == false)
	{
	  std::cerr << "Illegal link detected while linking audio." << "\n";
	  return;
	}

      gazebo::util::LinkAudio audio;

      if (_sdf->HasElement("audio_source"))
	{
	  sdf::ElementPtr audioElem = _sdf->GetElement("audio_source");
	  std::vector<std::string> collisionNames;

	  while (audioElem)
	    {
	      gazebo::util::OpenALSourcePtr source =
		gazebo::util::OpenAL::Instance()->CreateSource(audioElem);

	      std::vector<std::string> names = source->GetCollisionNames();
	      std::copy(names.begin(), names.end(), std::back_inserter(collisionNames));

	      audioElem = audioElem->GetNextElement("audio_source");
	      audio.audioSources.push_back(source);
	    }
	}

      // if (!collisionNames.empty())
      // 	{
      // 	  for (std::vector<std::string>::iterator iter = collisionNames.begin();
      // 	       iter != collisionNames.end(); ++iter)
      // 	    {
      // 	      (*iter) = this->GetChildLink(link)
      // 		->GetScopedName() + "::" + (*iter);
      // 	    }

      // 	  std::string topic =
      // 	    this->world->GetPhysicsEngine()->GetContactManager()
      // 	    ->CreateFilter(this->GetChildLink(link)->GetScopedName()
      // 			   + "/audio_collision", collisionNames);
      // 	}

      if (_sdf->HasElement("audio_sink"))
	{
	  audio.audioSink =
	    gazebo::util::OpenAL::Instance()->CreateSink(_sdf->GetElement("audio_sink"));
	}

      this->linkAudio[link] = audio;
    }

    void BVHactor::UpdateAudio()
    {
      for (std::map<std::string, gazebo::util::LinkAudio>::iterator iter =
	     this->linkAudio.begin(); iter != this->linkAudio.end(); ++iter)
	{
	  if (iter->second.audioSink)
	    {
	      iter->second.audioSink->SetPose(this->GetChildLink(iter->first)
					      ->GetWorldPose());
	      iter->second.audioSink->SetVelocity(this->GetChildLink(iter->first)
						  ->GetWorldLinearVel());
	    }

	  for (std::vector<gazebo::util::OpenALSourcePtr>::iterator it =
		 iter->second.audioSources.begin();
	       it != iter->second.audioSources.end(); ++it)
	    {
	      // if ((*it)->IsPlaying() == false)
	      // 	{
	      // 	  (*it)->Play();
	      // 	}
	      (*it)->SetPose(this->GetChildLink(iter->first)->GetWorldPose());
	      (*it)->SetVelocity(this->GetChildLink(iter->first)->GetWorldLinearVel());
	    }
	}
    }
#endif

  }
}
