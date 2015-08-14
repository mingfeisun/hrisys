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
	  this->playStartTime[limbs[i]] = this->world->GetSimTime();
	  this->lastScriptTime[limbs[i]] = std::numeric_limits<double>::max();
	  this->prevFrameTime[limbs[i]] = this->world->GetSimTime();
	  this->loop[limbs[i]] = false;
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
      std::string bvhFileName = std::string(va_arg(args, char*));
      this->skelAnim[limb] = _actor->GetSkelAnimationData(bvhFileName);
      if (this->skelAnim[limb] == nullptr)
	{
	  std::cerr << "Unexpected BVH file called in XBVHLimb" << "\n";
	  va_end(args);
	  return;
	}
      if (_actor->SetLimbMotion(limb, [=](XtendedActorPtr _a, std::string _s)
				{return this->UpdateLimbX(_a, _s);}) == false)
	{
	  std::cerr << "Unexpected limb called in XBVHLimb -2" << "\n";
	  va_end(args);
	  return;
	}
      this->playStartTime[limb] = this->world->GetSimTime();
      this->lastScriptTime[limb] = std::numeric_limits<double>::max();
      this->loop[limb] = static_cast<bool>(va_arg(args, int));
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
