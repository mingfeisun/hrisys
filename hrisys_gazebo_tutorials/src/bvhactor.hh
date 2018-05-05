#ifndef _HRI_BVHACTOR_H_
#define _HRI_BVHACTOR_H_

#include "gazebo/physics/Actor.hh"
#include "gazebo/util/OpenAL.hh"

namespace gazebo
{
  namespace physics
  {
    class BVHactor : public Model
    {
      public: explicit BVHactor(BasePtr _parent);
      public: virtual ~BVHactor();
      public: void Load(sdf::ElementPtr _sdf);
      public: void Fini();
      public: void AddBVHAnimation(sdf::ElementPtr _sdf);
      public: common::Skeleton* LoadBVH(const std::string &_filename,
                std::map<std::string, std::string> _skelMap,
                double _scale=0.0,
                bool _debug=false);
      public: void StartBVH(std::string bvhfile);
      public: void Update();
      protected: void MeshLoad(sdf::ElementPtr _sdf);
      protected: void LoadAnimation(sdf::ElementPtr _sdf);
      protected: void SetPose(std::map<std::string, math::Matrix4> _frame, double _time);

      protected: std::string bvhfileOnPlay;
      protected: bool doDebug;
      protected: std::map<std::string, math::Matrix4> debugPose;

      protected: const common::Mesh *mesh;
      protected: common::Skeleton *skeleton;
      protected: std::string skinFile;
      protected: double skinScale;
      protected: double startDelay;
      protected: double scriptLength;
      protected: double lastScriptTime;
      protected: bool loop;
      protected: bool active;
      protected: common::Time prevFrameTime;
      protected: common::Time playStartTime;
      protected: std::map<std::string, common::SkeletonAnimation*> skelAnimation;
      protected: std::map<std::string, std::map<std::string, std::string> > skelNodesMap;
      protected: std::string visualName;
      protected: uint32_t visualId;
      protected: transport::PublisherPtr bonePosePub;
    };

    typedef boost::shared_ptr<BVHactor> BVHactorPtr;
  }
}

#endif
