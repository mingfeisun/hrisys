#ifndef _HRI_XTENDEDACTOR_H_
#define _HRI_XTENDEDACTOR_H_

#include "gazebo/physics/Actor.hh"
#include "bvhactor.hh"

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
    public: explicit LimbXtentions(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~LimbXtentions();

    public: virtual void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
      /// Initializes UpdateLimbX and sets motion function for limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting limb motion.
      /// \param[in] _limbname Name of limb to set.
      /// \param[in] _arg Argument to set for fLimbNull function.
    public: virtual void StartLimbX(XtendedActorPtr _actor, std::string _limb, ...);

      /// \brief UpdateLimbX function.
    public: virtual void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: virtual void FinishLimbX(XtendedActorPtr _actor, std::string _limb);

    protected: WorldPtr world;

    protected: LimbXtentionsPtr next;
    };


    class XNullLimb : public LimbXtentions
    {
      /// \brief Constructor.
    public: explicit XNullLimb(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~XNullLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
    public: void StartLimbX(XtendedActorPtr _actor, std::string _limb, ...);

      /// \brief UpdateLimbX function. Apply no motion to nodes in limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting node poses.
      /// \param[in] _limbname Name of limb to set.
    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);
    };


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


    typedef boost::shared_ptr<XNullLimb> XNullLimbPtr;
    typedef boost::shared_ptr<XBvhLimb> XBvhLimbPtr;


    class XtendedActor : public BVHactor
    {
      /// \brief Constructor.
    public: explicit XtendedActor(BasePtr _parent);

      /// \brief Destructor.
    public: virtual ~XtendedActor();

      /// \brief Load the Xtended Actor from SDF.
    public: void Load(sdf::ElementPtr _sdf);

      /// \brief Update the Xteded Actor.
    public: void Update();

      /// \brief Get nodelist of limb.
      /// Used in UpdateLimbX functions. Public for UpdateLimbX extention.
      /// \param[in] _limb Name of limb.
      /// \return List of nodes in limb.
    public: std::vector<std::string> GetLimbNodeList(std::string _limb) const;

    public: math::Matrix4 GetNodePoseAtNow(std::string _node) const;

    //   /// \brief Get argument for UpdateLimbX function.
    //   /// Used in UpdateLimbX functions. Public for UpdateLimbX extention.
    //   /// \param[in] _limb Name of limb.
    //   /// \return String argument.
    // public: std::string GetLimbArg(std::string _limb);

    //   /// \brief Returns calculated frame matrices.
    //   /// Used in UpdateLimbX functions. Public for UpdateLimbX extention.
    //   /// \param[in] _bvhfile Name of BVH file to calculate.
    //   /// \return Matrices of current frame.
    // public: std::map<std::string, math::Matrix4> GetFrameOf(std::string _bvhfile,
    // 							    double _scripttime) const;

    public: common::SkeletonAnimation* GetSkelAnimationData(std::string _bvhfile) const;

    public: common::Skeleton* GetSkeletonData() const;

      /// \brief Set pose of node.
      /// Used in UpdateLimbX functions. Public for UpdateLimbX extention.
      /// \param[in] _node Name of node.
      /// \param[in] _pose Transform matrix of node.
    public: bool SetNodeTransform(std::string _node, math::Matrix4 _pose);

      /// \brief Set motion function for limb.
      /// Used in StartLimbX functions. Public for StartLimbX extention.
      /// \param[in] _limb Name of limb (node group).
      /// \param[in] _motion Motion function, functions from extention allowed.
    public: bool SetLimbMotion(std::string _limb,
			       std::function<void (XtendedActorPtr, std::string)> _motion);

    //   /// \brief Set argument of motion function for limb.
    //   /// Used in StartLimbX functions. Public for StartLimbX extention.
    //   /// \param[in] _limb Name of limb (node group).
    //   /// \param[in] _arg Argument to pass.
    // public: void SetLimbArg(std::string _limb, std::string _arg);

      /// \brief List of nodes in limb (node group).
    protected: std::map<std::string, std::vector<std::string> > limbNodeList;

      /// \brief UpdateLimbX function of limb.
    protected: std::map<std::string,
			std::function<void (XtendedActorPtr, std::string)> > limbMotion;

    //   /// \brief Stored argument for UpdateLimbX function of limb.
    //   /// Example of argument: filename used in UpdateLimbX function.
    //   /// String for flexibility. Argument could be "int int" if decoded in UpdateLimbX.
    // protected: std::map<std::string, std::string> limbArg;

      /// \brief Current pose of Xtended Actor.
    protected: std::map<std::string, math::Matrix4> poseAtNow;

    public: XNullLimbPtr xNull;
    };
  }
}

#endif
