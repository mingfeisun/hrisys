#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "XBvhLimb.hh"

namespace gazebo
{
  namespace physics
  {
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
	  XLimbAnimeManager limbAnimeManager;
	  limbAnimeManager.playStartTime = this->world->GetSimTime();
	  limbAnimeManager.lastScriptTime = std::numeric_limits<double>::max();
	  limbAnimeManager.prevFrameTime = this->world->GetSimTime();
	  limbAnimeManager.loop = false;
	  limbAnimeManager.scriptLength = 0;
	  limbAnimeManager.startDelay = 0;
	  limbAnimeManager.skelAnim = nullptr;
	  this->animeManager[limbs[i]] = limbAnimeManager;
	}
    }

    //////////////////////////////////////////////////
    void XBvhLimb::StartLimbX(XtendedActorPtr _actor,
			      std::string _limb, std::string _arg)
    {
      if (animeManager.find(_limb) == animeManager.end())
	{
	  std::cerr << "Unexpected limb called in XBVHLimb -1" << "\n";
	  return;
	}
      std::stringstream ss(_arg);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> args(begin, end);
      if (args.size() != 2)
	{
	  std::cerr << "Illegal argument in XBVHLimb" << "\n";
	  return;
	}
      std::string bvhFileName = args[0];
      this->animeManager[_limb].skelAnim = _actor->GetSkelAnimationData(bvhFileName);
      if (this->animeManager[_limb].skelAnim == nullptr)
	{
	  std::cerr << "Unexpected BVH file called in XBVHLimb" << "\n";
	  return;
	}
      if (_actor->SetLimbMotion(_limb, [=](XtendedActorPtr _a, std::string _s)
				{return this->UpdateLimbX(_a, _s);}) == false)
	{
	  std::cerr << "Unexpected limb called in XBVHLimb -2" << "\n";
	  return;
	}
      this->animeManager[_limb].playStartTime = this->world->GetSimTime();
      this->animeManager[_limb].lastScriptTime = std::numeric_limits<double>::max();
      this->animeManager[_limb].loop = std::stoi(args[1]);
      this->animeManager[_limb].scriptLength =
	this->animeManager[_limb].skelAnim->GetLength();
      /// set default next function to XNullLimb
      this->animeManager[_limb].next = _actor->xNull;
      this->animeManager[_limb].nextArg = "";
    }

    //////////////////////////////////////////////////
    void XBvhLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      /// note that whenever we want to inactivate an animation,
      /// we must set the limb function to next function
      this->animeManager[_limb].next
	->StartLimbX(_actor, _limb, this->animeManager[_limb].nextArg);
    }

    //////////////////////////////////////////////////
    void XBvhLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      common::Time currentTime = this->world->GetSimTime();
      XLimbAnimeManager limbAnimeManager = this->animeManager[_limb];

      this->animeManager[_limb].prevFrameTime = currentTime;

      double scriptTime = currentTime.Double()
	- limbAnimeManager.startDelay
	- limbAnimeManager.playStartTime.Double();

      /// waiting for delayed start
      if (scriptTime < 0)
	return;

      if (scriptTime >= limbAnimeManager.scriptLength)
	{
	  if (!limbAnimeManager.loop)
	    {
	      FinishLimbX(_actor, _limb);
	      return;
	    }
	  else
	    {
	      scriptTime = scriptTime - limbAnimeManager.scriptLength;
	      this->animeManager[_limb].playStartTime =
		currentTime - scriptTime;
	    }
	}

      /// get node posture
      std::map<std::string, math::Matrix4> frame;
      frame = limbAnimeManager.skelAnim->GetPoseAt(scriptTime);

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

      this->animeManager[_limb].lastScriptTime = scriptTime;
    }

  }
}
