#ifndef _HRI_XTENDEDACTOR_H_
#define _HRI_XTENDEDACTOR_H_

#include "gazebo/physics/Actor.hh"
#include "bvhactor.hh"
#include "LimbXtentions.hh"

namespace gazebo
{
  namespace physics
  {
    class XtendedActor;
    typedef boost::shared_ptr<XtendedActor> XtendedActorPtr;
    class LimbXtentions;
    typedef boost::shared_ptr<LimbXtentions> LimbXtentionsPtr;

    class XNullLimb : public LimbXtentions
    {
      /// \brief Constructor.
    public: explicit XNullLimb(WorldPtr _world);

      /// \brief Destructor.
    public: virtual ~XNullLimb();

    public: void InitLimbX(sdf::ElementPtr _sdf);

      /// \brief StartLimbX function.
    public: void StartLimbX(XtendedActorPtr _actor,
			    std::string _limb, std::string _arg);

      /// \brief UpdateLimbX function. Apply no motion to nodes in limb.
      /// \param[in] _actor Pointer to Xtended Actor for setting node poses.
      /// \param[in] _limbname Name of limb to set.
    public: void UpdateLimbX(XtendedActorPtr _actor, std::string _limb);

    public: void FinishLimbX(XtendedActorPtr _actor, std::string _limb);
    };


    typedef boost::shared_ptr<XNullLimb> XNullLimbPtr;


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

      /// \brief List of nodes in limb (node group).
    protected: std::map<std::string, std::vector<std::string> > limbNodeList;

      /// \brief UpdateLimbX function of limb.
    protected: std::map<std::string,
			std::function<void (XtendedActorPtr, std::string)> > limbMotion;

      /// \brief Current pose of Xtended Actor.
    protected: std::map<std::string, math::Matrix4> poseAtNow;

    public: XNullLimbPtr xNull;
    };
  }
}

#endif
