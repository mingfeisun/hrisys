#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "ActionTrigger.hh"
#include "XBvhLimb.hh"
#include "XWalkerLimb.hh"

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    KeyBoardWalkerTrigger::KeyBoardWalkerTrigger() : ActionTrigger()
    {
      value = math::Vector3(0, 0, 0);
    }

    //////////////////////////////////////////////////
    KeyBoardWalkerTrigger::~KeyBoardWalkerTrigger()
    {
    }

    //////////////////////////////////////////////////
    void KeyBoardWalkerTrigger::TriggerProcess()
    {
      static struct termios oldt, newt;
      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
      fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

      int c = getchar();

      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      fcntl(STDIN_FILENO, F_SETFL, oldf);

      if (c == EOF)
	{
	  if ((this->state == TriggerState::T_ON) ||
	      (this->state == TriggerState::T_FIRE))
	    this->state = TriggerState::T_RELEASE;
	  else
	    this->state = TriggerState::T_OFF;
	  return;
	}

      if (c == 'a')
	value = math::Vector3(0.0, 0.1, 0.0);
      if (c == 'd')
	value = math::Vector3(0.0, -0.1, 0.0);
      if (c == 's')
	value = math::Vector3(-0.1, 0.0, 0.0);
      if (c == 'w')
	value = math::Vector3(0.1, 0.0, 0.0);

      if ((this->state == TriggerState::T_OFF) ||
	  (this->state == TriggerState::T_RELEASE))
	this->state = TriggerState::T_FIRE;
      else
	this->state = TriggerState::T_ON;
      return;
    }


    //////////////////////////////////////////////////
    XWalkerLimb::XWalkerLimb(WorldPtr _world) : XBvhLimb(_world)
    {
    }

    //////////////////////////////////////////////////
    XWalkerLimb::~XWalkerLimb()
    {
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
      XBvhLimb::InitLimbX(_sdf);

      std::string triggerType = _sdf->Get<std::string>("trigger");
      if (triggerType == "keyboard")
	this->trigger.reset(new KeyBoardWalkerTrigger());

      this->paramT = 0.0;
      this->deltaT = 0.0;
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::StartLimbX(XtendedActorPtr _actor,
				 std::string _limb, std::string _arg)
    {
      XBvhLimb::StartLimbX(_actor, _limb, _arg);
      for (unsigned int i = 0; i < _actor->GetSkeletonData()->GetNumNodes(); ++i)
      	{
      	  common::SkeletonNode* node =
      	    _actor->GetSkeletonData()->GetNodeByHandle(i);
      	  this->frame0[node->GetName()] = node->GetTransform();
      	}
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::PassIdleMotion(std::map<std::string, math::Matrix4> _frame)
    {
      for (auto iter = _frame.begin(); iter != _frame.end(); ++iter)
	{
	  auto it = this->frame0.find(iter->first);
	  if (it != this->frame0.end())
	    it->second = iter->second;
	}
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::UpdateLimbX(XtendedActorPtr _actor,
				  std::string _limb)
    {
      this->trigger->TriggerProcess();

      if (this->trigger->GetTriggerState() == TriggerState::T_FIRE)
	  this->deltaT = 0.05;
      else if (this->trigger->GetTriggerState() == TriggerState::T_OFF)
	  this->deltaT = -0.05;

      this->paramT += this->deltaT;

      if (this->paramT < 0.05)
	this->paramT = 0.0;
      else if (this->paramT > 0.95)
	this->paramT = 1.0;
      
      this->moveDistance += this->paramT * this->trigger->GetValue();

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
	  scriptTime = scriptTime - limbAnimeManager.scriptLength;
	  this->animeManager[_limb].playStartTime =
	    currentTime - scriptTime;
	}

      /// get node posture
      std::map<std::string, math::Matrix4> frame;

      /// get node list
      std::vector<std::string> limbnodelist = _actor->GetLimbNodeList(_limb);

      /// the limb should include the root
      if (find(limbnodelist.begin(), limbnodelist.end(),
	       _actor->GetSkeletonData()->GetRootNode()->GetName()) !=
	  limbnodelist.end())
	{
	  frame = limbAnimeManager.skelAnim->
	    GetPoseAtX(this->moveDistance.GetLength(),
		       _actor->GetSkeletonData()->GetRootNode()->GetName());
	  math::Matrix4 rootTrans =
	    frame[_actor->GetSkeletonData()->GetRootNode()->GetName()];
	  math::Vector3 rootPos = rootTrans.GetTranslation();
	  math::Quaternion rootRot = rootTrans.GetRotation();
	  math::Pose modelPose;
	  math::Pose actorPose;
	  actorPose.pos = modelPose.pos + modelPose.rot.RotateVector(rootPos);
	  actorPose.rot = modelPose.rot * rootRot;
	  math::Vector3 velocity = this->trigger->GetValue();
	  double theta;
	  if (fabs(velocity.x) < 0.001)
	    if (fabs(velocity.y) < 0.001)
	      theta = 0;
	    else
	      theta = (velocity.y / fabs(velocity.y)) * M_PI / 2;
	  else
	    theta = atan(velocity.y / velocity.x);
	  math::Quaternion direction(math::Vector3(0, 0, 1), theta);
	  actorPose.rot = direction * actorPose.rot;
	  math::Matrix4 rootM(actorPose.rot.GetAsMatrix4());
	  rootM.SetTranslate(math::Vector3(this->moveDistance.x,
					   this->moveDistance.y,
					   actorPose.pos.z));
	  frame[_actor->GetSkeletonData()->GetRootNode()->GetName()] = rootM;
	}
      else
	{
	  std::cerr << "Error! XWalker must include root! \n";
	  this->FinishLimbX(_actor, _limb);
	  return;
	}

      /// set node posture
      for (int i = 0; i < limbnodelist.size(); ++i)
	{
	  if (limbnodelist[i] == _actor->GetSkeletonData()->GetRootNode()->GetName())
	    {
	      _actor->SetNodeTransform(limbnodelist[i], frame[limbnodelist[i]]);
	      continue;
	    }

	  math::Matrix4 slerpFrame =
	    math::Quaternion::Slerp(this->paramT,
				    this->frame0[limbnodelist[i]].GetRotation(),
				    frame[limbnodelist[i]].GetRotation()).GetAsMatrix4();
	  slerpFrame.SetTranslate(frame[limbnodelist[i]].GetTranslation());
	  _actor->SetNodeTransform(limbnodelist[i], slerpFrame);
	}

      this->animeManager[_limb].lastScriptTime = scriptTime;
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::FinishLimbX(XtendedActorPtr _actor,
				  std::string _limb)
    {
      XBvhLimb::FinishLimbX(_actor, _limb);
    }
  }
}
