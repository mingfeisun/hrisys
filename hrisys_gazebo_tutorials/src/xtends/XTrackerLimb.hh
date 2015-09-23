#ifndef _XTENTIONS_XTRACKERLIMB_H_
#define _XTENTIONS_XTRACKERLIMB_H_

# ifdef HRISYS_HAVE_OPENNI

#include <NiTE.h>
#include "../LimbXtentions.hh"

#include "../utils/UncertainMemoryFilter.hh"
#include "../utils/GreedyDecisionFilter.hh"

namespace gazebo
{
  namespace physics
  {
    enum MapJointType
      {
	J_ROOT,

	J_STATIC,

	J_TRANSLATE,

	J_ROTATE
      };

    class XLimbSkeletonJoint
    {
    public: std::string nameInLimb;

    public: nite::JointType nameInNite;

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

    public: bool trackOnly;

    public: UncertainMemoryFilterPtr<math::Vector3> longFilter;

    public: GreedyDecisionFilterPtr<math::Vector3> shortFilter;
    };

    class XLimbActorSkeletonManager
    {
    public: int userId;

    public: bool registered;

    public: std::string limb;

    public: std::vector<XLimbSkeletonJoint> joints;

    public: LimbXtentionsPtr next;

    public: std::string nextArg;

    public: bool rootInitialized;

    public: math::Vector3 rootOrigin;

    public: float rootScale;

    public: bool longFilterOn;

    public: bool shortFilterOn;

    public: std::map<std::string, math::Matrix4> trackedFrameData;
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

    public: std::map<std::string, math::Matrix4>
    GetTrackedFrameData(std::string _name) const;

    protected: std::map<std::string, XLimbActorSkeletonManager> skelManager;

    protected: nite::UserTracker userTracker;

    protected: nite::UserTrackerFrameRef userTrackerFrame;

    protected: std::string headActor;

    protected: int onRegister;

    protected: bool doTrack;

    public: std::function<double(math::Vector3, math::Vector3)> quadratic;

    private: class XTrackerDevel;

    private: XTrackerDevel *develPtr;
    };

    typedef boost::shared_ptr<XTrackerLimb> XTrackerLimbPtr;
  }
}

# endif

#endif
