#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"

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
      this->value.onTransparent = false;

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
      if (c == 'c')
	{
	  this->value.onTransparent = true;
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

      if (msg->nunchuk_buttons[0] > 0.1 && this->zCount == 0) /// Z button
	{
	  this->value.resetCamera = true;
	  ++this->zCount;
	}
      else
	{
	  this->value.resetCamera = false;
	  this->zCount = 0;
	}

      if (msg->buttons[5]) this->value.onTransparent = true; /// B button
      else this->value.onTransparent = false;
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

      this->node.reset(new transport::Node());
      this->node->Init();
      this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 200);
      this->visPub->WaitForConnection();
      this->onAppear = true;
      this->visualized.resize(10, true);

      if (_sdf->HasElement("camera"))
	{
	  sdf::ElementPtr cameraSdf = _sdf->GetElement("camera");
	  this->cameraPose = cameraSdf->Get<math::Pose>("pose");
	  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui", 5);
	  this->guiPub->WaitForConnection();
	  this->cameraEnabled = true;

	  double width, range;
	  if (cameraSdf->HasElement("width")) width = cameraSdf->Get<double>("width");
	  else width = 5.0;

	  if (cameraSdf->HasElement("range")) range = cameraSdf->Get<double>("range");
	  else range = -0.5;

	  if (cameraSdf->HasElement("sizeThreshold"))
	    this->sizeThreshold = cameraSdf->Get<double>("sizeThreshold");
	  else this->sizeThreshold = 1.0;

	  math::Vector3 lA(this->cameraPose.pos.x,
			   this->cameraPose.pos.y - width/2, 0);
	  lA = lA.Normalize();
	  math::Vector3 lB(this->cameraPose.pos.x,
			   this->cameraPose.pos.y + width/2, 0);
	  lB = lB.Normalize();
	  this->lineAIni = math::Pose(lA, this->cameraPose.rot);
	  this->lineBIni = math::Pose(lB, this->cameraPose.rot);
	  this->lineLIni = math::Pose(math::Vector3(range, 0, 0), this->cameraPose.rot);

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

      /// handle model transparent
      if (this->trigger->GetValue().onTransparent)
	{
	  if (this->onAppear == true)
	    {
	      msgs::Visual visualMsg;
	      visualMsg.set_name(_actor->GetScopedName());
	      visualMsg.set_parent_name(_actor->GetParent()->GetScopedName());
	      visualMsg.set_transparency(0.8);
	      this->visPub->Publish(visualMsg);
	      this->onAppear = false;
	    }
	}
      else
	{
	  if (this->onAppear == false)
	    {
	      msgs::Visual visualMsg;
	      visualMsg.set_name(_actor->GetScopedName());
	      visualMsg.set_parent_name(_actor->GetParent()->GetScopedName());
	      visualMsg.set_transparency(0.0);
	      this->visPub->Publish(visualMsg);
	      this->onAppear = true;
	    }
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
	      this->deltaCameraT = 0.2; /// for keyboard control try 0.01
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

	  /// handle visibility of models in front of camera
	  /// calculate current disable-visibility-region boundary lines lineA, lineB
	  math::Vector3 lineA = this->globalDirection * this->lineAIni.pos;
	  math::Vector3 lineB = this->globalDirection * this->lineBIni.pos;
	  /// pointL: the disable boarder point, lineL: the disable boarder horizon
	  math::Vector3 pointL = this->globalDirection * this->lineLIni.pos;
	  math::Vector3 lineL = this->globalDirection *
	    (this->lineLIni.pos + math::Vector3(0, 1, 0)) - pointL;
	  /// on which side of line the camera exists
	  double sideCameraIsOnA = this->OnSide(lineA, this->cameraPose.pos);
	  double sideCameraIsOnB = this->OnSide(lineB, this->cameraPose.pos);
	  /// on which side of boarder the model exists
	  double sideActorIsOnL = this->OnSide(lineL, -pointL);

	  /// in case new objects are added
	  if (this->visualized.size() < this->world->GetModelCount())
	    this->visualized.resize(this->world->GetModelCount(), true);

	  for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
	    {
	      ModelPtr model = this->world->GetModel(i);
	      if (model->GetName() == _actor->GetName())
		continue;

	      math::Box objectBound = model->GetBoundingBox();
	      double px = objectBound.GetXLength() / 2;
	      double py = objectBound.GetYLength() / 2;
	      math::Vector3 centerPos = objectBound.GetCenter() -
		math::Vector3(this->globalPosition.x, this->globalPosition.y, 0);

	      /// only change visibility of those that are big and "get in the way"
	      if (objectBound.GetZLength() < this->sizeThreshold)
		continue;

	      math::Vector3 object2Camera = this->cameraPose.pos - centerPos;
	      double object2CameraSquared =
		object2Camera.x * object2Camera.x + object2Camera.y * object2Camera.y;
	      double object2BoundSquared = px * px + py * py;

	      /// indication of object boundary point (larger than actual boundary)
	      math::Vector3 boundPos =
		centerPos + object2BoundSquared * object2Camera;
	      double sideObjectBoundaryIsOnA = this->OnSide(lineA, boundPos);
	      double sideObjectBoundaryIsOnB = this->OnSide(lineB, boundPos);

	      /// when object boundary point is in disable-visibility-region
	      if (sideObjectBoundaryIsOnA * sideCameraIsOnA > 0 &&
		  sideObjectBoundaryIsOnB * sideCameraIsOnB > 0)
		{
		  /// only refresh visibilty when visibility state flips
		  if (this->visualized[i] == true)
		    {
		      double sideCorner1IsOnL =
			this->OnSide(lineL,
				     centerPos + math::Vector3(px, py, 0) - pointL);
		      double sideCorner2IsOnL =
			this->OnSide(lineL,
				     centerPos + math::Vector3(-px, py, 0) - pointL);
		      double sideCorner3IsOnL =
			this->OnSide(lineL,
				     centerPos + math::Vector3(-px, -py, 0) - pointL);
		      double sideCorner4IsOnL =
			this->OnSide(lineL,
				     centerPos + math::Vector3(px, -py, 0) - pointL);

		      /// don't disable when at least one of the corners are
		      /// out of disable boarder, or else long objects will
		      /// be disabled from the scene
		      if (sideCorner1IsOnL * sideActorIsOnL > 0 ||
			  sideCorner2IsOnL * sideActorIsOnL > 0 ||
			  sideCorner3IsOnL * sideActorIsOnL > 0 ||
			  sideCorner4IsOnL * sideActorIsOnL > 0)
			continue;

		      msgs::Visual visualMsg;
		      visualMsg.set_name(model->GetScopedName());
		      visualMsg.set_parent_name(model->GetParent()->GetScopedName());
		      visualMsg.set_visible(false);
		      this->visPub->Publish(visualMsg);
		      this->visualized[i] = false;
		      break; /// only disable one object at a time (crashes)
		    }
		  continue;
		}

	      /// only refresh visibilty when visibility state flips
	      if (this->visualized[i] == false)
		{
		  msgs::Visual visualMsg;
		  visualMsg.set_name(model->GetScopedName());
		  visualMsg.set_parent_name(model->GetParent()->GetScopedName());
		  visualMsg.set_visible(true);
		  this->visPub->Publish(visualMsg);
		  this->visualized[i] = true;
		  /// visualize visualizes COM and collision, so disable them
		  transport::requestNoReply(this->world->GetName(), "hide_com", "all");
		  transport::requestNoReply(this->world->GetName(),
					    "hide_collision", "all");
		  break; /// only visualize one object at a time (crashes)
		}
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
