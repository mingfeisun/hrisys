#ifndef _XTENTIONS_XLEGKINEMATICSLIMB_H_
#define _XTENTIONS_XLEGKINEMATICSLIMB_H_

#include "../LimbXtentions.hh"
#include "XTrackerLimb.hh"

namespace gazebo
{
  namespace physics
  {
    class XLimbLegSkeletonJoint
    {
    public: std::string nameInLimb;

    public: math::Vector3 translate;

    public: math::Matrix4 priorFixer;

    public: math::Matrix4 posteriorFixer;

    public: bool calculateOnly;
    };

    class XLimbLegSkeletonManager
    {
    public: std::vector<XLimbLegSkeletonJoint> joints;

    public: LimbXtentionsPtr next;

    public: std::string nextArg;

    public: std::vector<double> links;

    public: std::map<std::string, math::Matrix4> legFrame;
    };

    class XLegKinematicsLimb : public LimbXtentions
    {
    public: explicit XLegKinematicsLimb(WorldPtr _world);

    public: virtual ~XLegKinematicsLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

    public: void StartLimbX(XtendedActorPtr _actor,
			    std::string _limb, std::string _arg);

    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

# ifdef HRISYS_HAVE_OPENNI
    public: void SetFromTracker(XTrackerLimbPtr _tracker);
# endif

    protected: std::map<std::string, XLimbLegSkeletonManager> skelManager;

    protected: XTrackerLimbPtr fromTracker;

    protected: bool getFromXTracker;
    };
  }
}

#endif
