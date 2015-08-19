#ifndef _HRI_LIMBXTENTIONS_H_
#define _HRI_LIMBXTENTIONS_H_

namespace gazebo
{
  namespace physics
  {
    class XtendedActor;
    typedef boost::shared_ptr<XtendedActor> XtendedActorPtr;
    class LimbXtentions;
    typedef boost::shared_ptr<LimbXtentions> LimbXtentionsPtr;

    class LimbXtentions
    {
      /// \brief Constructor.
    public: explicit LimbXtentions(WorldPtr _world) {this->world = _world;};

      /// \brief Destructor.
    public: virtual ~LimbXtentions() {};

    public: virtual void InitLimbX(sdf::ElementPtr _sdf) = 0;

      /// \brief StartLimbX function.
      /// Initializes UpdateLimbX and sets motion function for limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting limb motion.
      /// \param[in] _limb Name of limb to set.
      /// \param[in] _arg Argument to set for fLimbNull function.
    public: virtual void StartLimbX(XtendedActorPtr _actor,
				    std::string _limb, std::string _arg) = 0;

      /// \brief UpdateLimbX function.
    public: virtual void UpdateLimbX(XtendedActorPtr _actor, std::string _limb) = 0;

    public: virtual void FinishLimbX(XtendedActorPtr _actor, std::string _limb) = 0;

    protected: WorldPtr world;
    };
  }
}

#endif
