#ifndef _XTENTIONS_XTRACKERLIMB_H_
#define _XTENTIONS_XTRACKERLIMB_H_

# ifdef HRISYS_HAVE_OPENNI

#include <NiTE.h>
#include "../LimbXtentions.hh"

namespace gazebo
{
  namespace physics
  {
    enum MapJointType
      {
	J_STATIC,

	J_TRANSLATE,

	J_ROTATE
      };

    class XLimbSkeletonJoint
    {
    public: std::string nameInLimb;

    public: nite::JointType nameInNite;

      /// will be deprecated
    public: nite::JointType parentInNite;

    public: nite::JointType childInNite;

    public: math::Vector3 translate;

    public: math::Vector3 childTranslate;

    public: math::Matrix4 priorFixer;

    public: math::Matrix4 posteriorFixer;

    public: MapJointType type;

      /// currently not used
    public: unsigned int confidentNTimes;

    public: bool trackJoint;
    };

    class XLimbActorSkeletonManager
    {
    public: int userId;

    public: bool registered;

    public: std::string limb;

    public: std::vector<XLimbSkeletonJoint> joints;

    public: LimbXtentionsPtr next;

    public: std::string nextArg;
    };

    class XTrackerLimb : public LimbXtentions
    {
      /// \brief Constructor.
    public: explicit XTrackerLimb(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~XTrackerLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
    public: void StartLimbX(XtendedActorPtr _actor,
			    std::string _limb, std::string _arg);

      /// \brief UpdateLimbX function. Set motion of limb by tracking.
    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

    protected: std::map<std::string, XLimbActorSkeletonManager> skelManager;

    protected: nite::UserTracker userTracker;

    protected: nite::UserTrackerFrameRef userTrackerFrame;

    protected: std::string headActor;

    protected: int onRegister;

    protected: bool doTrack;
    };

    typedef boost::shared_ptr<XTrackerLimb> XTrackerLimbPtr;
  }
}

# endif

#endif
