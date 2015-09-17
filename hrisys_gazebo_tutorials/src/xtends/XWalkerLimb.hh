#ifndef _XTENTIONS_XWALKERLIMB_H_
#define _XTENTIONS_XWALKERLIMB_H_

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "../LimbXtentions.hh"
#include "ActionTrigger.hh"
#include "XBvhLimb.hh"

# ifdef HRISYS_HAVE_ROS
#include <ros/ros.h>
#include "wiimote/State.h"
# endif

namespace gazebo
{
  namespace physics
  {
    struct WalkerTrigger
    {
      /// \brief Model moving direction in local coordinate.
      math::Vector3 pos;

      /// \brief View control flag.
      bool resetCamera;
    };

    class KeyBoardWalkerTrigger : public ActionTrigger<WalkerTrigger>
    {
    public: KeyBoardWalkerTrigger();

    public: virtual ~KeyBoardWalkerTrigger();

    public: void TriggerProcess();
    };

    typedef boost::shared_ptr<KeyBoardWalkerTrigger> KeyBoardWalkerTriggerPtr;

# ifdef HRISYS_HAVE_ROS
    class WiiWalkerTrigger : public ActionTrigger<WalkerTrigger>
    {
    public: WiiWalkerTrigger();

    public: virtual ~WiiWalkerTrigger();

    public: void TriggerProcess();

    private: void Callback(const wiimote::State::ConstPtr& msg);

    private: ros::NodeHandle nh;

    private: ros::Subscriber sub;

      /// \brief X input by nunchuk.
    private: float moveX;

      /// \brief Y input by nunchuk.
    private: float moveY;

      /// \brief Trigger check of Z button.
    private: int zCount;
    };

    typedef boost::shared_ptr<WiiWalkerTrigger> WiiWalkerTriggerPtr;
# endif

    class XWalkerLimb : public XBvhLimb
    {
    public: explicit XWalkerLimb(WorldPtr _world);

    public: virtual ~XWalkerLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

    public: void StartLimbX(XtendedActorPtr _actor,
			    std::string _limb, std::string _arg);

    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

      /// \brief Handles value and state by player control.
    protected: ActionTriggerPtr<WalkerTrigger> trigger;

      /// \brief Initial non-walking frame matrices.
    protected: std::map<std::string, math::Matrix4> frame0;

      /// \brief Position of model in global coordinate.
    protected: math::Vector3 globalPosition;

      /// \brief Continously moved absolute distance.
      /// Resets to zero once the model stops moving.
    protected: math::Vector3 moveDistance;

      /// \brief Direction the model is facing.
      /// The direction is defined in global coordinate.
      /// Note: This is not always equal to the camera direction.
    protected: math::Quaternion direction;

      /// \brief Returns the angle to calculate direction.
    protected: std::function<double()> theta;

      /// \brief The transition speed from
      /// non-walking to walking motion.
    protected: float deltaT;

      /// \brief Linear transition parameter between
      /// non-walking and walking motion.
      /// Non-walks when paramT is 0.0, and walks when 1.0.
    protected: float paramT;

      /// \brief Node that handles communication to camera.
    protected: transport::NodePtr node;

      /// \brief Publisher to communicate to camera.
    protected: transport::PublisherPtr guiPub;

      /// \brief Relative pose of camera from model.
    protected: math::Pose cameraPoseIni;

      /// \brief Current position of camera in global coordinate.
    protected: math::Pose cameraPose;

      /// \brief Direction the camera was facing.
      /// Pror view in view-change-interpolation.
    protected: math::Quaternion globalDirectionFrom;

      /// \brief Direction the camera is facing.
    protected: math::Quaternion globalDirection;

      /// \brief Direction the camera is going to face.
      /// Posterior view in view-change-interpolation.
    protected: math::Quaternion globalDirectionTo;

      /// \brief The interpolation speed in view change.
    protected: float deltaCameraT;

      /// \brief Linear transition parameter between
      /// prior view and posterior view.
    protected: float cameraT;

      /// \brief View control by player flag.
      /// Triggers when reDirection < 0. When triggered,
      /// model direction and camera direction is matched.
    protected: float reDirection;

      /// \brief Count up to change camera direction.
      /// Triggers when no activity is done by player. When
      /// triggered, model and camera direction starts matching.
    protected: int autoCameraSetCount;

      /// \brief Settings for using camera.
      /// When false, camera is independent (world view).
      /// When true, camera is linked to model (player view).
    protected: bool cameraEnabled;
    };

    typedef boost::shared_ptr<XWalkerLimb> XWalkerLimbPtr;
  }
}

#endif
