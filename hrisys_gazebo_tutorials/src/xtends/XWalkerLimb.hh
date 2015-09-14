#ifndef _XTENTIONS_XWALKERLIMB_H_
#define _XTENTIONS_XWALKERLIMB_H_

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
    class KeyBoardWalkerTrigger : public ActionTrigger<math::Vector3>
    {
    public: KeyBoardWalkerTrigger();

    public: virtual ~KeyBoardWalkerTrigger();

    public: void TriggerProcess();
    };

    typedef boost::shared_ptr<KeyBoardWalkerTrigger> KeyBoardWalkerTriggerPtr;

# ifdef HRISYS_HAVE_ROS
    class WiiWalkerTrigger : public ActionTrigger<math::Vector3>
    {
    public: WiiWalkerTrigger();

    public: virtual ~WiiWalkerTrigger();

    public: void TriggerProcess();

    private: void Callback(const wiimote::State::ConstPtr& msg);

    private: ros::NodeHandle nh;

    private: ros::Subscriber sub;

    private: float moveX;

    private: float moveY;
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

    public: void PassIdleMotion(std::map<std::string, math::Matrix4> _frame);

    protected: ActionTriggerPtr<math::Vector3> trigger;

    protected: std::map<std::string, math::Matrix4> frame0;

    protected: math::Vector3 moveDistance;

    protected: float deltaT;

    protected: float paramT;
    };

    typedef boost::shared_ptr<XWalkerLimb> XWalkerLimbPtr;
  }
}

#endif
