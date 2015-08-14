#ifndef _XTENTIONS_XBVHLIMB_H_
#define _XTENTIONS_XBVHLIMB_H_

#include "../LimbXtentions.hh"

namespace gazebo
{
  namespace physics
  {
    class XBvhLimb : public LimbXtentions
    {
      /// \brief Constructor.
    public: explicit XBvhLimb(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~XBvhLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
    public: void StartLimbX(XtendedActorPtr _actor, std::string _limb, ...);

      /// \brief UpdateLimbX function. Apply BVH motion to nodes in limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting node poses.
      /// \param[in] _limbname Name of limb to set.
    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

      // protected: std::map<std::string, std::string> bvhFileName;

    protected: std::map<std::string, common::SkeletonAnimation*> skelAnim;

      /// \brief True if the animation should loop.
    protected: std::map<std::string, bool> loop;

      /// \brief Time of the previous frame.
    protected: std::map<std::string, common::Time> prevFrameTime;

      /// \brief Time when the animation was started.
    protected: std::map<std::string, common::Time> playStartTime;

      /// \brief Amount of time to delay start by.
    protected: std::map<std::string, double> startDelay;

      /// \brief Time length of a scipt.
    protected: std::map<std::string, double> scriptLength;

      /// \brief Time the scipt was last updated.
    protected: std::map<std::string, double> lastScriptTime;
    };

    typedef boost::shared_ptr<XBvhLimb> XBvhLimbPtr;
  }
}

#endif
