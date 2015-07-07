#ifndef _HRI_PLUGIN_
#define _HRI_PLUGIN_

#include "gazebo/physics/Actor.hh"

namespace gazebo
{
  namespace physics
  {
    class BVHactor : public Actor
    {
    public: explicit BVHactor(BasePtr _parent);
    public: virtual ~BVHactor();
    public: void Load(sdf::ElementPtr _sdf);
    public: void AddBVHAnimation(sdf::ElementPtr _sdf);
    public: common::Skeleton* LoadBVH(const std::string &_filename,
				      std::map<std::string, std::string> _skelMap,
				      double _scale=0.0,
				      bool _debug=false);
    public: void StartBVH(std::string bvhfile);
    public: void Update();
    protected: void SetPose(std::map<std::string, math::Matrix4> _frame, double _time);

    protected: std::string bvhfileOnPlay;

    protected: bool doDebug;
    protected: std::map<std::string, math::Matrix4> debugPose;
    };

    typedef boost::shared_ptr<BVHactor> BVHactorPtr;
  }
}

#endif
