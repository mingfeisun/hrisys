#ifndef _HRI_BVHACTOR_H_
#define _HRI_BVHACTOR_H_

#include "gazebo/physics/Actor.hh"
#include "gazebo/util/OpenAL.hh"

namespace gazebo
{
  namespace util
  {
# ifdef HRISYS_HAVE_OPENAL
    class LinkAudio
    {
    public: LinkAudio() {};

      /// \brief All the audio sources
    public: std::vector<OpenALSourcePtr> audioSources;

      /// \brief An audio sink
    public: OpenALSinkPtr audioSink;

      /// \brief Subscriber to contacts with this collision. Used for audio
      /// playback.
    // public: gazebo::transport::SubscriberPtr audioContactsSub;
    };
# endif
  }

  namespace physics
  {
    class BVHactor : public Actor
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
    protected: void SetPose(std::map<std::string, math::Matrix4> _frame, double _time);
# ifdef HRISYS_HAVE_OPENAL
    public: void AddAudioToLink(sdf::ElementPtr _sdf);
    public: void UpdateAudio();

    protected: std::map<std::string, gazebo::util::LinkAudio> linkAudio;
# endif
    protected: std::string bvhfileOnPlay;
    protected: bool doDebug;
    protected: std::map<std::string, math::Matrix4> debugPose;
    };

    typedef boost::shared_ptr<BVHactor> BVHactorPtr;
  }
}

#endif
