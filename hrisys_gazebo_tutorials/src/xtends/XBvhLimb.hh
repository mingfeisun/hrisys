#ifndef _XTENTIONS_XBVHLIMB_H_
#define _XTENTIONS_XBVHLIMB_H_

#include "../LimbXtentions.hh"

namespace gazebo
{
  namespace physics
  {
    class XLimbAnimeManager
    {
    public: common::SkeletonAnimation* skelAnim;

      /// \brief True if the animation should loop.
    public: bool loop;

      /// \brief Time of the previous frame.
    public: common::Time prevFrameTime;

      /// \brief Time when the animation was started.
    public: common::Time playStartTime;

      /// \brief Amount of time to delay start by.
    public: double startDelay;

      /// \brief Time length of a scipt.
    public: double scriptLength;

      /// \brief Time the scipt was last updated.
    public: double lastScriptTime;

    public: LimbXtentionsPtr next;

    public: std::string nextArg;
    };


    class XBvhLimb : public LimbXtentions
    {
      /// \brief Constructor.
    public: explicit XBvhLimb(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~XBvhLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
    public: void StartLimbX(XtendedActorPtr _actor,
			    std::string _limb, std::string _arg);

      /// \brief UpdateLimbX function. Apply BVH motion to nodes in limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting node poses.
      /// \param[in] _limb Name of limb to set.
    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

    protected: std::map<std::string, XLimbAnimeManager> animeManager;
    };

    typedef boost::shared_ptr<XBvhLimb> XBvhLimbPtr;
  }
}

#endif
