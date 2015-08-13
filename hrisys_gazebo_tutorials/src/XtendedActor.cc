#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/PhysicsIface.hh"

#include "XtendedActor.hh"

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    XtendedActor::XtendedActor(BasePtr _parent) : BVHactor(_parent)
    {
      xNull.reset(new XNullLimb(this->world));
    };

    //////////////////////////////////////////////////
    XtendedActor::~XtendedActor() {};

    //////////////////////////////////////////////////
    void XtendedActor::Load(sdf::ElementPtr _sdf)
    {
      BVHactor::Load(_sdf);

      /// add limbs
      if (_sdf->HasElement("limb"))
	{
	  /// create a check list for duplicated nodes
	  std::map<std::string, bool> nodeCheckList;
	  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
	    nodeCheckList[this->skeleton->GetNodeByHandle(i)->GetName()] = false;

	  sdf::ElementPtr limbSdf = _sdf->GetElement("limb");
	  while (limbSdf)
	    {
	      std::string limbName = limbSdf->Get<std::string>("name");
	      std::vector<std::string> nodeList;
	      sdf::ElementPtr nodeSdf = _sdf->GetElement("node");
	      while (nodeSdf)
		{
		  std::string nodeName = nodeSdf->Get<std::string>("name");
		  if (nodeCheckList.find(nodeName) != nodeCheckList.end())
		    {
		      /// the node exists
		      if (nodeCheckList[nodeName] == true)
			/// node is already classified in a different limb
			std::cerr << "Duplicated node " << nodeName
				  << " detected in limb " << limbName << "\n";
		      else
			nodeList.push_back(nodeName);
		      nodeCheckList[nodeName] = true;
		    }
		  nodeSdf = nodeSdf->GetNextElement("node");
		}
	      this->limbNodeList[limbName] = nodeList;
	      limbSdf = limbSdf->GetNextElement("limb");
	    }

	  /// check if all nodes were classified
	  for (std::map<std::string, bool>::iterator iter =
		 nodeCheckList.begin(); iter != nodeCheckList.end(); ++iter)
	    if (iter->second == false)
	      std::cerr << "Node " << iter->first
			<< " was not found in any limb" << "\n";
	}

      /// set default motions to NULL
      for (std::map<std::string, std::vector<std::string> >::iterator iter=
	     limbNodeList.begin(); iter != limbNodeList.end(); ++iter)
	xNull->StartLimbX(boost::static_pointer_cast<XtendedActor>(shared_from_this()),
			  iter->first);

      /// set when xtended actor is not an ordinary bvh player
      if (bvhfileOnPlay == "")
	{
	  this->playStartTime = this->world->GetSimTime();
	  this->lastScriptTime = std::numeric_limits<double>::max();
	}
    }

    //////////////////////////////////////////////////
    void XtendedActor::Update()
    {
      if (this->autoStart)
	{
	  Actor::Update();
	  return;
	}

      if (this->bvhfileOnPlay != "")
	{
	  BVHactor::Update();
	  return;
	}

      common::Time currentTime = this->world->GetSimTime();

      /// do not refresh animation more faster the 30 Hz sim time
      /// TODO: Reducing to 20 Hz. Because there were memory corruption
      /// and segmentation faults. Possibly due to some dangling pointers
      /// in pose message processing. This will need a proper fix. Just a
      /// workaround for now.
      if ((currentTime - this->prevFrameTime).Double() < (1.0 / 20.0))
	return;

      /// at this point we are certain that a new frame will be animated
      this->prevFrameTime = currentTime;

      /// set poseAtNow of each node according to defined limb motion
      for (std::map<std::string, std::function<void (XtendedActorPtr,std::string)> >::iterator
	     iter = limbMotion.begin(); iter != limbMotion.end(); ++iter)
	limbMotion[iter->first](boost::static_pointer_cast<XtendedActor>(shared_from_this()),
				iter->first);

      /// send to gzclient the current pose
      this->SetPose(poseAtNow, currentTime.Double());
    }

    //////////////////////////////////////////////////
    std::vector<std::string> XtendedActor::GetLimbNodeList(std::string _limb) const
    {
      std::map<std::string, std::vector<std::string> >::const_iterator iter
	= limbNodeList.find(_limb);

      if (iter == limbNodeList.end())
	return std::vector<std::string>();
      return iter->second;
    }

    //////////////////////////////////////////////////
    math::Matrix4 XtendedActor::GetNodePoseAtNow(std::string _node) const
    {
      std::map<std::string, math::Matrix4>::const_iterator iter
	= poseAtNow.find(_node);

      if (iter == poseAtNow.end())
	return math::Matrix4();
      return iter->second;
    }

    //////////////////////////////////////////////////
    // std::string XtendedActor::GetLimbArg(std::string _limb)
    // {
    //   if (limbArg.find(_limb) == limbArg.end())
    // 	return std::string();
    //   return limbArg[_limb];
    // }

    //////////////////////////////////////////////////
    // std::map<std::string, math::Matrix4> XtendedActor::GetFrameOf
    // (std::string _bvhfile, double _scripttime) const
    // {
    //   std::map<std::string, common::SkeletonAnimation*>::const_iterator iter
    // 	= this->skelAnimation.find(_bvhfile);
    //   if (iter == this->skelAnimation.end())
    // 	return std::map<std::string, math::Matrix4>();
    //   common::SkeletonAnimation *skelAnim = iter->second;
    //   return skelAnim->GetPoseAt(_scripttime);
    // }

    //////////////////////////////////////////////////
    common::SkeletonAnimation* XtendedActor::GetSkelAnimationData(std::string _bvhfile) const
    {
      std::map<std::string, common::SkeletonAnimation*>::const_iterator iter
	= this->skelAnimation.find(_bvhfile);
      if (iter == this->skelAnimation.end())
	return nullptr;
      return iter->second;
    }

    //////////////////////////////////////////////////
    common::Skeleton* XtendedActor::GetSkeletonData() const
    {
      return this->skeleton;
    }

    //////////////////////////////////////////////////
    bool XtendedActor::SetNodeTransform(std::string _node, math::Matrix4 _pose)
    {
      if (poseAtNow.find(_node) == poseAtNow.end())
	return false;
      poseAtNow[_node] = _pose;
      return true;
    }

    //////////////////////////////////////////////////
    bool XtendedActor::SetLimbMotion
    (std::string _limb, std::function<void (XtendedActorPtr, std::string)> _motion)
    {
      if (limbMotion.find(_limb) == limbMotion.end())
	return false;
      limbMotion[_limb] = _motion;
      return true;
    }

    //////////////////////////////////////////////////    
    // void XtendedActor::SetLimbArg(std::string _limb, std::string arg)
    // {
    //   if (limbArg.find(_limb) == limbArg.end())
    // 	return;
    //   limbArg[_limb] = _arg;
    // }



    //////////////////////////////////////////////////
    LimbXtentions::LimbXtentions(WorldPtr _world)
    {
      this->world = _world;
    }

    //////////////////////////////////////////////////
    LimbXtentions::~LimbXtentions()
    {
    }



    //////////////////////////////////////////////////
    XNullLimb::XNullLimb(WorldPtr _world) : LimbXtentions(_world)
    {
    }

    //////////////////////////////////////////////////
    XNullLimb::~XNullLimb()
    {
    }
    
    //////////////////////////////////////////////////
    void XNullLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
    }

    //////////////////////////////////////////////////
    void XNullLimb::StartLimbX(XtendedActorPtr _actor, std::string _limb, ...)
    {
      if (_actor->SetLimbMotion(_limb, [=](XtendedActorPtr _a, std::string _s)
				{return this->UpdateLimbX(_a, _s);}) == false)
	std::cerr << "Unexpected limb called in XNullLimb" << "\n";
    }

    //////////////////////////////////////////////////
    void XNullLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      /// get node list
      std::vector<std::string> limbnodelist = _actor->GetLimbNodeList(_limb);

      /// return node posture of last frame
      for (int i = 0; i < limbnodelist.size(); ++i)
	_actor->SetNodeTransform(limbnodelist[i], _actor->GetNodePoseAtNow(limbnodelist[i]));
    }

    //////////////////////////////////////////////////
    void XNullLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
    }



    //////////////////////////////////////////////////
    XBvhLimb::XBvhLimb(WorldPtr _world) : LimbXtentions(_world)
    {
    }

    //////////////////////////////////////////////////
    XBvhLimb::~XBvhLimb()
    {
    }

    //////////////////////////////////////////////////
    void XBvhLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
      /// set time managers for each limb
      std::stringstream ss(_sdf->Get<std::string>("limbs"));
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> limbs(begin, end);
      for (unsigned int i = 0; i < limbs.size(); ++i)
	{
	  this->playStartTime[limbs[i]] = this->world->GetSimTime();
	  this->lastScriptTime[limbs[i]] = std::numeric_limits<double>::max();
	  this->prevFrameTime[limbs[i]] = this->world->GetSimTime();
	  this->loop[limbs[i]] = false;
	  // this->bvhFileName[limbs[i]] = "";
	  this->scriptLength[limbs[i]] = 0;
	  this->startDelay[limbs[i]] = 0;
	  this->skelAnim[limbs[i]] = nullptr;
	}
    }

    //////////////////////////////////////////////////
    void XBvhLimb::StartLimbX(XtendedActorPtr _actor, std::string _limb, ...)
    {
      std::string limb = _limb;
      if (skelAnim.find(limb) == skelAnim.end())
	{
	  std::cerr << "Unexpected limb called in XBVHLimb -1" << "\n";
	  return;
	}
      va_list args;
      va_start(args, _limb);
      // this->bvhFileName[limb] = std::string(va_arg(args, char*));
      std::string bvhFileName = std::string(va_arg(args, char*));
      this->skelAnim[limb] = _actor->GetSkelAnimationData(bvhFileName);
      if (this->skelAnim[limb] == nullptr)
	{
	  std::cerr << "Unexpected BVH file called in XBVHLimb" << "\n";
	  return;
	}
      if (_actor->SetLimbMotion(limb, [=](XtendedActorPtr _a, std::string _s)
				{return this->UpdateLimbX(_a, _s);}) == false)
	{
	  std::cerr << "Unexpected limb called in XBVHLimb -2" << "\n";
	  return;
	}
      this->playStartTime[limb] = this->world->GetSimTime();
      this->lastScriptTime[limb] = std::numeric_limits<double>::max();
      this->loop[limb] = va_arg(args, bool);
      va_end(args);
      this->scriptLength[limb] = this->skelAnim[limb]->GetLength();
      /// set default next function to XNullLimb
      next = _actor->xNull;
    }

    //////////////////////////////////////////////////
    void XBvhLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      /// note that whenever we want to inactivate an animation,
      /// we must set the limb function to next function
      next->StartLimbX(_actor, _limb);
    }

    //////////////////////////////////////////////////
    void XBvhLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      common::Time currentTime = this->world->GetSimTime();

      this->prevFrameTime[_limb] = currentTime;

      double scriptTime = currentTime.Double() - this->startDelay[_limb] -
	this->playStartTime[_limb].Double();

      /// waiting for delayed start
      if (scriptTime < 0)
	return;

      if (scriptTime >= this->scriptLength[_limb])
	{
	  if (!this->loop[_limb])
	    {
	      FinishLimbX(_actor, _limb);
	      return;
	    }
	  else
	    {
	      scriptTime = scriptTime - this->scriptLength[_limb];
	      this->playStartTime[_limb] = currentTime - scriptTime;
	    }
	}

      /// get node posture
      std::map<std::string, math::Matrix4> frame;
      // frame = _actor->GetFrameOf(bvhFileName[_limb], scriptTime);
      // if (frame.empty())
      // 	return;
      frame = skelAnim[_limb]->GetPoseAt(scriptTime);

      /// get node list
      std::vector<std::string> limbnodelist = _actor->GetLimbNodeList(_limb);

      /// if limbNodeList which includes root
      if (find(limbnodelist.begin(), limbnodelist.end(),
	       _actor->GetSkeletonData()->GetRootNode()->GetName())
	  != limbnodelist.end())
	{
	  math::Matrix4 rootTrans =
	    frame[_actor->GetSkeletonData()->GetRootNode()->GetName()];
	  math::Vector3 rootPos = rootTrans.GetTranslation();
	  math::Quaternion rootRot = rootTrans.GetRotation();
	  math::Pose modelPose;
	  math::Pose actorPose;
	  actorPose.pos = modelPose.pos + modelPose.rot.RotateVector(rootPos);
	  actorPose.rot = modelPose.rot * rootRot;
	  math::Matrix4 rootM(actorPose.rot.GetAsMatrix4());
	  rootM.SetTranslate(actorPose.pos);
	  frame[_actor->GetSkeletonData()->GetRootNode()->GetName()] = rootM;
	}

      /// set node posture
      for (int i = 0; i < limbnodelist.size(); ++i)
	_actor->SetNodeTransform(limbnodelist[i], frame[limbnodelist[i]]);

      this->lastScriptTime[_limb] = scriptTime;
    }


  }
}
