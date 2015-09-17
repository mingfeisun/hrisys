#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "ActionTrigger.hh"
#include "XBvhLimb.hh"
#include "XWalkerLimb.hh"

# ifdef HRISYS_HAVE_ROS
#include <ros/ros.h>
#include "wiimote/State.h"
# endif

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    KeyBoardWalkerTrigger::KeyBoardWalkerTrigger() : ActionTrigger()
    {
      this->value.pos = math::Vector3(0, 0, 0);
      this->value.resetCamera = false;
    }

    //////////////////////////////////////////////////
    KeyBoardWalkerTrigger::~KeyBoardWalkerTrigger()
    {
    }

    //////////////////////////////////////////////////
    void KeyBoardWalkerTrigger::TriggerProcess()
    {
      this->value.resetCamera = false;

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

      if (c == 'a') this->value.pos = math::Vector3(0.0, 0.1, 0.0);
      if (c == 'd') this->value.pos = math::Vector3(0.0, -0.1, 0.0);
      if (c == 's') this->value.pos = math::Vector3(-0.1, 0.0, 0.0);
      if (c == 'w') this->value.pos = math::Vector3(0.1, 0.0, 0.0);

      if (c == 'f')
	{
	  this->value.resetCamera = true;
	  return;
	}

      if ((this->state == TriggerState::T_OFF) ||
	  (this->state == TriggerState::T_RELEASE))
	this->state = TriggerState::T_FIRE;
      else
	this->state = TriggerState::T_ON;
      return;
    }


# ifdef HRISYS_HAVE_ROS
    //////////////////////////////////////////////////
    WiiWalkerTrigger::WiiWalkerTrigger() : ActionTrigger(), nh()
    {
      this->moveX = 0.0;
      this->moveY = 0.0;
      this->zCount = 0;
      this->value.pos = math::Vector3(0, 0, 0);
      this->value.resetCamera = false;

      if (!ros::isInitialized())
	{
	  ROS_FATAL_STREAM("ROS initialization error!");
	  return;
	}

      this->sub = nh.subscribe("/wiimote/state", 1,
			       &WiiWalkerTrigger::Callback, this);
    }

    //////////////////////////////////////////////////
    WiiWalkerTrigger::~WiiWalkerTrigger()
    {
    }

    //////////////////////////////////////////////////
    void WiiWalkerTrigger::TriggerProcess()
    {
      if (moveX == 0.0 && moveY == 0.0)
	{
	  if ((this->state == TriggerState::T_ON) ||
	      (this->state == TriggerState::T_FIRE))
	    this->state = TriggerState::T_RELEASE;
	  else
	    this->state = TriggerState::T_OFF;
	  return;
	}

      this->value.pos = math::Vector3(moveX, moveY, 0.0);

      if ((this->state == TriggerState::T_OFF) ||
	  (this->state == TriggerState::T_RELEASE))
	this->state = TriggerState::T_FIRE;
      else
	this->state = TriggerState::T_ON;
      return;
    }

    //////////////////////////////////////////////////
    void WiiWalkerTrigger::Callback(const wiimote::State::ConstPtr& msg)
    {
      float x = fabs(msg->nunchuk_joystick_zeroed[1]);
      float y = fabs(msg->nunchuk_joystick_zeroed[0]);
      float sgnx = 0.0, sgny = 0.0;
      if (x > 0.0) sgnx = msg->nunchuk_joystick_zeroed[1] / x;
      if (y > 0.0) sgny = msg->nunchuk_joystick_zeroed[0] / y;

      if (x > 0.8) this->moveX = sgnx * 0.1;
      else if (x > 0.2) this->moveX = sgnx * x * 0.1;
      else this->moveX = 0.0;

      if (y > 0.8) this->moveY = sgny * 0.1;
      else if (y > 0.2) this->moveY = sgny * y * 0.1;
      else this->moveY = 0.0;

      if (msg->nunchuk_buttons[0] > 0.1 && this->zCount == 0)
	{
	  this->value.resetCamera = true;
	  ++this->zCount;
	}
      else
	{
	  this->value.resetCamera = false;
	  this->zCount = 0;
	}
    }
# endif


    //////////////////////////////////////////////////
    XWalkerLimb::XWalkerLimb(WorldPtr _world) : XBvhLimb(_world)
    {
      this->cameraEnabled = false;
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
# ifdef HRISYS_HAVE_ROS
      if (triggerType == "wii")
	this->trigger.reset(new WiiWalkerTrigger());
# endif

      this->paramT = 0.0;
      this->deltaT = 0.0;

      this->globalDirectionFrom = math::Quaternion(0, 0, 0);
      this->globalDirectionTo = this->globalDirectionFrom;
      this->globalDirection = this->globalDirectionTo;
      this->reDirection = 1.0;

      this->theta = [=]()
	{
	  math::Vector3 velocity = this->trigger->GetValue().pos;
	  double theta;
	  if (fabs(velocity.x) < 0.001)
	    if (fabs(velocity.y) < 0.001)
	      theta = 0;
	    else
	      theta = (velocity.y / fabs(velocity.y)) * M_PI / 2;
	  else
	    theta = atan(velocity.y / velocity.x)
	      - (velocity.x / fabs(velocity.x) - 1) * M_PI / 2;
	  return theta;
	};

      if (_sdf->HasElement("camera"))
	{
	  sdf::ElementPtr cameraSdf = _sdf->GetElement("camera");
	  this->cameraPose = cameraSdf->Get<math::Pose>("pose");
	  this->node.reset(new transport::Node());
	  this->node->Init();
	  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui", 5);
	  this->guiPub->WaitForConnection();
	  this->cameraEnabled = true;

	  this->cameraT = 1.0;
	  this->autoCameraSetCount = 0;
	}
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

      if (this->cameraEnabled)
	{
	  math::Pose pose =
	    _actor->GetChildLink(_actor->GetSkeletonData()->GetRootNode()
				 ->GetName())->GetWorldCoGPose();
	  this->cameraPose += math::Pose(0, 0, pose.pos.z, 0, 0, 0);
	  this->cameraPoseIni = this->cameraPose;
	  pose = this->cameraPose + math::Pose(pose.pos.x, 0, 0, 0, 0, 0);
	  msgs::GUI result;
	  msgs::GUICamera *guiCam = result.mutable_camera();
	  guiCam->set_name("user_camera");
	  guiCam->set_view_controller("orbit");
	  msgs::Set(guiCam->mutable_pose(), pose);
	  this->guiPub->Publish(result);
	}
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::UpdateLimbX(XtendedActorPtr _actor,
				  std::string _limb)
    {
      this->trigger->TriggerProcess();

      if (this->trigger->GetTriggerState() == TriggerState::T_FIRE)
	{
	  this->deltaT = 0.05;
	  this->moveDistance = 0.0;
	}
      else if (this->trigger->GetTriggerState() == TriggerState::T_OFF)
	{
	  this->deltaT = -0.05;
	}

      this->paramT += this->deltaT;

      if (this->paramT < 0.05)
	this->paramT = 0.0;
      else if (this->paramT > 0.95)
	this->paramT = 1.0;
      
      math::Vector3 deltaD =
	this->paramT * (this->globalDirection * this->trigger->GetValue().pos);

      this->moveDistance += deltaD;
      this->globalPosition += deltaD;

      /// get node posture
      std::map<std::string, math::Matrix4> frame;

      /// get node list
      std::vector<std::string> limbnodelist = _actor->GetLimbNodeList(_limb);

      /// the limb should include the root
      if (find(limbnodelist.begin(), limbnodelist.end(),
	       _actor->GetSkeletonData()->GetRootNode()->GetName()) ==
	  limbnodelist.end())
	{
	  std::cerr << "Error! XWalker must include root! \n";
	  this->FinishLimbX(_actor, _limb);
	  return;
	}

      XLimbAnimeManager limbAnimeManager = this->animeManager[_limb];

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

      /// when model is moving
      if (this->trigger->GetTriggerState() == TriggerState::T_FIRE ||
	  this->trigger->GetTriggerState() == TriggerState::T_ON)
	{
	  /// when interrupted during camera view change
	  if (this->reDirection > 0.99 && this->cameraT < 1)
	    {
	      this->globalDirectionTo = this->globalDirection;
	      this->cameraT = 1.0;
	    }

	  this->direction =
	    this->globalDirection * math::Quaternion(math::Vector3(0, 0, 1),
						     this->theta());
	}
      /// when change view is triggered
      else if (this->reDirection < 0)
	{
	  this->reDirection = 1.0;
	  double theta = this->theta();
	  if (fabs(theta) < M_PI / 2)
	    this->direction =
	      this->globalDirection * math::Quaternion(math::Vector3(0, 0, 1), theta);
	}

      actorPose.rot = this->direction * actorPose.rot;
      math::Matrix4 rootM(actorPose.rot.GetAsMatrix4());
      rootM.SetTranslate(math::Vector3(this->globalPosition.x,
				       this->globalPosition.y,
				       actorPose.pos.z));
      frame[_actor->GetSkeletonData()->GetRootNode()->GetName()] = rootM;

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

      /// handle camera view
      if (this->cameraEnabled)
	{
	  if (this->trigger->GetTriggerState() == TriggerState::T_OFF)
	    ++this->autoCameraSetCount;
	  /// when model is moving
	  else
	    this->autoCameraSetCount = 0;

	  /// camera view change is triggered
	  if (this->trigger->GetValue().resetCamera)
	    {
	      this->globalDirectionFrom = this->globalDirectionTo;
	      this->globalDirectionTo = this->direction;
	      this->cameraT = 0.0;
	      this->deltaCameraT = 0.2;
	      this->autoCameraSetCount = -20;
	      this->reDirection = 1.0;
	    }
	  /// after model stops moving
	  else if (this->autoCameraSetCount == 10)
	    {
	      this->globalDirectionFrom = this->globalDirectionTo;
	      this->globalDirectionTo = this->direction;
	      this->cameraT = 0.0;
	      this->deltaCameraT = 0.2;
	      this->reDirection = 100.0; /// do not reDirection
	    }

	  /// calculate camera view
	  if (this->cameraT < 1)
	    {
	      this->cameraT += this->deltaCameraT;
	      this->globalDirection =
		math::Quaternion::Slerp(this->cameraT,
					this->globalDirectionFrom,
					this->globalDirectionTo);
	      this->cameraPose =
		math::Pose(this->globalDirection * this->cameraPoseIni.pos,
			   this->globalDirection);
	      this->reDirection -= this->cameraT;
	    }

	  /// change camera view
	  math::Pose pose =
	    this->cameraPose + math::Pose(this->globalPosition.x,
					  this->globalPosition.y, 0, 0, 0, 0);
	  msgs::GUI result;
	  msgs::GUICamera *guiCam = result.mutable_camera();
	  guiCam->set_name("user_camera");
	  guiCam->set_view_controller("orbit");
	  msgs::Set(guiCam->mutable_pose(), pose);
	  this->guiPub->Publish(result);
	}
    }

    //////////////////////////////////////////////////
    void XWalkerLimb::FinishLimbX(XtendedActorPtr _actor,
				  std::string _limb)
    {
      XBvhLimb::FinishLimbX(_actor, _limb);
    }
  }
}
