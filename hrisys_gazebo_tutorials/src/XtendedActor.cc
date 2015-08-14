#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/common/Skeleton.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Actor.hh"

#include "XtendedActor.hh"
#include "LimbXtentions.hh"

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
	      sdf::ElementPtr nodeSdf = limbSdf->GetElement("node");
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
	  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
	    poseAtNow[this->skeleton->GetNodeByHandle(i)->GetName()]
	      = this->skeleton->GetNodeByHandle(i)->GetTransform();
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
      if (limbNodeList.find(_limb) == limbNodeList.end())
	return false;
      limbMotion[_limb] = _motion;
      return true;
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

  }
}
