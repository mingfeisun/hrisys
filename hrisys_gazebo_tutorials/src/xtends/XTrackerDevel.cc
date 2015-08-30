#ifdef HRISYS_HAVE_OPENNI

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <NiTE.h>

#include "gazebo/physics/World.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "XTrackerLimb.hh"

#define NUM_OF_NITE_JOINTS 22

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    struct TestSample {
      double x;
      double y;
      double z;
      double qx;
      double qy;
      double qz;
      double qw;
    };

    //////////////////////////////////////////////////
    class XTrackerLimb::XTrackerDevel
    {
    public: std::string checkFile;

    public: int from;

    public: int to;

    public: unsigned int count;

    public: std::vector<std::map<nite::JointType, TestSample> > samples;
    };

    //////////////////////////////////////////////////
    XTrackerLimb::XTrackerLimb(WorldPtr _world) : LimbXtentions(_world)
    {
      {{
	  develPtr = new XTrackerDevel();
      }}
    }

    //////////////////////////////////////////////////
    XTrackerLimb::~XTrackerLimb()
    {
      {{
	  delete develPtr;
      }}
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
      struct XLimbTemplateJoint
      {
      public: nite::JointType nameInNite;
      public: MapJointType type;
      public: unsigned int nodeId;
      };

      unsigned int nodeIdCount = 0;
      std::map<std::string, XLimbTemplateJoint> templateSkelJoints;
      std::vector<common::SkeletonNode*> nodes;
      nodes.resize(NUM_OF_NITE_JOINTS);

      auto setNewJoint = [&](std::string name, std::string parent,
			     nite::JointType niteType, MapJointType type,
			     double x, double y, double z)
        { nodeIdCount++;
	  nodes[nodeIdCount] =
          new common::SkeletonNode(nodes[templateSkelJoints[parent].nodeId],
                                   name, name, common::SkeletonNode::JOINT);
	  nodes[nodeIdCount]
          ->SetTransform((math::Matrix4(1, 0, 0, x,
                                        0, 1, 0, y,
                                        0, 0, 1, z,
                                        0, 0, 0, 1)), false);
	  templateSkelJoints[name] = {niteType, type, nodeIdCount}; };

      templateSkelJoints["ROOT"] = {nite::JOINT_TORSO, J_STATIC};
      nodes[templateSkelJoints["ROOT"].nodeId] =
	new common::SkeletonNode(NULL, "ROOT", "ROOT",
				 common::SkeletonNode::JOINT);
      nodes[templateSkelJoints["ROOT"].nodeId]
      	->SetTransform(math::Matrix4::IDENTITY, false);

      setNewJoint("TORSO", "ROOT", nite::JOINT_TORSO, J_ROTATE, 0, 0, 0);
      setNewJoint("CHEST", "TORSO", nite::JOINT_TORSO, J_STATIC, 0, 1, 0);
      setNewJoint("VIRTUAL_NECK", "CHEST", nite::JOINT_NECK, J_TRANSLATE, 0, 0, 0);
      setNewJoint("NECK", "VIRTUAL_NECK", nite::JOINT_NECK, J_ROTATE, 0, 1, 0);
      setNewJoint("HEAD", "NECK", nite::JOINT_HEAD, J_ROTATE, 0, 1, 0);
      setNewJoint("VIRTUAL_LEFT_SHOULDER", "CHEST",
      		  nite::JOINT_LEFT_SHOULDER, J_TRANSLATE, 0, 0, 0);
      setNewJoint("LEFT_SHOULDER", "VIRTUAL_LEFT_SHOULDER",
      		  nite::JOINT_LEFT_SHOULDER, J_ROTATE, 1, 0, 0);
      setNewJoint("LEFT_ELBOW","LEFT_SHOULDER", nite::JOINT_LEFT_ELBOW, J_ROTATE, 1, 0, 0);
      setNewJoint("LEFT_HAND", "LEFT_ELBOW", nite::JOINT_LEFT_HAND, J_STATIC, 1, 0, 0);
      setNewJoint("VIRTUAL_RIGHT_SHOULDER", "CHEST",
      		  nite::JOINT_RIGHT_SHOULDER, J_TRANSLATE, 0, 0, 0);
      setNewJoint("RIGHT_SHOULDER", "VIRTUAL_RIGHT_SHOULDER",
      		  nite::JOINT_RIGHT_SHOULDER, J_ROTATE, -1, 0, 0);
      setNewJoint("RIGHT_ELBOW", "RIGHT_SHOULDER",
      		  nite::JOINT_RIGHT_ELBOW, J_ROTATE, -1, 0, 0);
      setNewJoint("RIGHT_HAND", "RIGHT_ELBOW", nite::JOINT_RIGHT_HAND, J_STATIC, -1, 0, 0);
      setNewJoint("VIRTUAL_LEFT_HIP", "ROOT", nite::JOINT_LEFT_HIP, J_TRANSLATE, 0, 0, 0);
      setNewJoint("LEFT_HIP", "VIRTUAL_LEFT_HIP",
      		  nite::JOINT_LEFT_HIP, J_ROTATE, 0.707, -0.707, 0);
      setNewJoint("LEFT_KNEE", "LEFT_HIP", nite::JOINT_LEFT_KNEE, J_ROTATE, 0, -1, 0);
      setNewJoint("LEFT_FOOT", "LEFT_KNEE", nite::JOINT_LEFT_FOOT, J_STATIC, 0, -1, 0);
      setNewJoint("VIRTUAL_RIGHT_HIP", "ROOT", nite::JOINT_RIGHT_HIP, J_TRANSLATE, 0, 0, 0);
      setNewJoint("RIGHT_HIP", "VIRTUAL_RIGHT_HIP",
      		  nite::JOINT_RIGHT_HIP, J_ROTATE, -0.707, -0.707, 0);
      setNewJoint("RIGHT_KNEE", "RIGHT_HIP", nite::JOINT_RIGHT_KNEE, J_ROTATE, 0, -1, 0);
      setNewJoint("RIGHT_FOOT", "RIGHT_KNEE", nite::JOINT_RIGHT_FOOT, J_STATIC, 0, -1, 0);

      std::stringstream ss(_sdf->Get<std::string>("actors"));
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> actors(begin, end);

      for (unsigned int i = 0; i < actors.size(); ++i)
        {
          XLimbActorSkeletonManager actorSkelManager;

          XtendedActorPtr actor =
            boost::static_pointer_cast<XtendedActor>(this->world->GetByName(actors[i]));

	  std::map<std::string, std::string> skelMap = actor->GetSkelMap("mapOpenNI");
          if (skelMap.empty())
            {
	      std::cerr << actors[i] << " does not have mapOpenNI." << "\n";
              continue;
            }
          if (skelMap.size() != nodes.size())
            {
	      std::cerr << "mapOpenNI has illegal number of joints." << "\n";
              continue;
            }

	  std::map<std::string, std::string> rSkelMap;
          for (std::map<std::string, std::string>::iterator iter =
                 skelMap.begin(); iter != skelMap.end(); ++iter)
            {
              rSkelMap[iter->second] = iter->first;
            }

          bool errorDetectedInCheck = false;
	  std::map<std::string, math::Matrix4> translationAligner;
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];
	      common::SkeletonNode *nodeInActor =
                actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()]);

              if (!node || !nodeInActor)
                {
		  std::cerr << "mapOpenNI has illegal joint name : "
			    << node->GetName() << " or "
			    << nodeInActor->GetName() << "\n";
                  errorDetectedInCheck = true;
                  break;
                }

              if (node->GetChildCount() != nodeInActor->GetChildCount())
		  if (node->GetChildCount() != 0)
		    {
		      std::cerr << "mapOpenNI has illegal joint pair : "
				<< node->GetName() << " and "
				<< nodeInActor->GetName() << "\n";
		      errorDetectedInCheck = true;
		      break;
		    }

	      math::Matrix4 transform(node->GetTransform());
	      math::Vector3 niteOffset = node->GetTransform().GetTranslation();
	      math::Vector3 daeOffset = nodeInActor->GetTransform().GetTranslation();
              transform.SetTranslate(daeOffset.GetLength() * niteOffset.Normalize());
              node->SetTransform(transform, false);
	    }

          if (errorDetectedInCheck)
            continue;

          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];
	      common::SkeletonNode *nodeInActor =
                actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()]);

              if (node->GetParent() != NULL)
                {
                  if (node->GetParent()->GetChildCount() > 1)
                    {
                      if (node->GetTransform().GetTranslation() ==
                          math::Vector3(0, 0, 0))
                        translationAligner[node->GetName()] = math::Matrix4::IDENTITY;
                    }
                }

              if (node->GetChildCount() == 0)
                continue;

              if (node->GetName() ==
                  skelMap[actor->GetSkeletonData()->GetRootNode()->GetName()])
                {
                  translationAligner[node->GetName()] =
                    actor->GetSkeletonData()->GetRootNode()
                    ->GetTransform().GetRotation().GetAsMatrix4();
		  math::Matrix4 tmp = translationAligner[node->GetName()];
                  tmp.SetTranslate(node->GetTransform().GetTranslation());
                  node->SetTransform(tmp, true);
                }

              if (node->GetChildCount() > 1)
                continue;

	      math::Vector3 relativeBVH =
                node->GetChild(0)->GetModelTransform().GetTranslation()
                - node->GetModelTransform().GetTranslation();
	      math::Vector3 relativeDAE =
                nodeInActor->GetChild(0)->GetModelTransform().GetTranslation()
                - nodeInActor->GetModelTransform().GetTranslation();

              if (relativeDAE == math::Vector3(0, 0, 0))
                {
		  std::cerr << "WARNING! Weird joints in model?" << std::endl;
                  continue;
                }

	      math::Vector3 n = relativeBVH.Cross(relativeDAE);
              double theta =
                asin(n.GetLength() / (relativeDAE.GetLength() * relativeBVH.GetLength()));

              translationAligner[node->GetChild(0)->GetName()]
                = nodeInActor->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();

	      math::Matrix4 tmp =
                node->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();
              tmp.SetTranslate(node->GetTransform().GetTranslation());
              node->SetTransform(tmp, true);
	    }

	  std::map<std::string, math::Matrix4> rotationAligner;
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];

              if (node->GetName()
		  == skelMap[actor->GetSkeletonData()->GetRootNode()->GetName()])
                {
                  rotationAligner[node->GetName()] = math::Matrix4::IDENTITY;
                  continue;
                }

              if (translationAligner[node->GetName()] == math::Matrix4::ZERO)
                translationAligner[node->GetName()] = math::Matrix4::IDENTITY;

              rotationAligner[node->GetName()]
                = node->GetTransform().GetRotation().GetAsMatrix4().Inverse()
                * translationAligner[node->GetName()].Inverse()
                * actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()])
                ->GetTransform().GetRotation().GetAsMatrix4();
            }

          actorSkelManager.joints.resize(NUM_OF_NITE_JOINTS);
          for (std::map<std::string, XLimbTemplateJoint>::iterator iter =
                 templateSkelJoints.begin();
               iter != templateSkelJoints.end(); ++iter)
            {
	      common::SkeletonNode* thisJoint = nodes[(iter->second).nodeId];
	      common::SkeletonNode* parentJoint = thisJoint->GetParent();
	      common::SkeletonNode* childJoint = NULL;

	      while (parentJoint != NULL)
		{
		  if ((iter->second).nameInNite !=
		      templateSkelJoints[parentJoint->GetName()].nameInNite)
		    break;
		  parentJoint = parentJoint->GetParent();
		}
	      if (parentJoint == NULL)
		parentJoint = thisJoint;

	      if (thisJoint->GetChildCount() == 1)
		{
		  childJoint = thisJoint->GetChild(0);
		  while (childJoint != NULL)
		    {
		      if ((iter->second).nameInNite !=
			  templateSkelJoints[childJoint->GetName()].nameInNite)
			break;
		      childJoint = childJoint->GetChild(0);
		    }
		}
	      if (childJoint == NULL)
		childJoint = thisJoint;

              XLimbSkeletonJoint joint;
              joint.nameInLimb = rSkelMap[iter->first];
              joint.nameInNite = (iter->second).nameInNite;
	      joint.parentInNite = templateSkelJoints[parentJoint->GetName()].nameInNite;
	      joint.childInNite = templateSkelJoints[childJoint->GetName()].nameInNite;
              joint.type = (iter->second).type;
	      joint.translate = thisJoint->GetTransform().GetTranslation();
	      joint.childTranslate = childJoint->GetTransform().GetTranslation();
              joint.priorFixer = translationAligner[iter->first];
              joint.posteriorFixer = rotationAligner[iter->first];
              joint.confidentNTimes = 0;
              joint.trackJoint = false;
              actorSkelManager.joints[(iter->second).nodeId] = joint;
            }

	  {{
	      std::cout << "/////////////////////////////////////////" << std::endl;
	      std::cout << "Showing debug: " << std::endl;

	      for (unsigned int i = 0; i < nodes.size(); ++i)
		{
		  common::SkeletonNode *node = nodes[i];
		  math::Matrix4 transform(math::Matrix4::IDENTITY);
		  transform.SetTranslate(node->GetTransform().GetTranslation());
		  transform =
		    translationAligner[node->GetName()]
		    * transform * rotationAligner[node->GetName()];
		  std::cout << node->GetName() << std::endl;
		  std::cout << "position align matrix" << std::endl;
		  std::cout << translationAligner[node->GetName()];
		  std::cout << "rotation align matrix" << std::endl;
		  std::cout << rotationAligner[node->GetName()];
		  std::cout << "dae" << std::endl;
		  std::cout << actor->GetSkeletonData()
		    ->GetNodeByName(rSkelMap[node->GetName()])->GetTransform();
		  std::cout << "bvh" << std::endl;
		  std::cout << transform << std::endl;
		}

	      std::cout << "/////////////////////////////////////////" << std::endl;
	  }}

          this->skelManager[actors[i]] = actorSkelManager;
	}


      {{
	  if (_sdf->HasElement("devel"))
	    {
	      sdf::ElementPtr develSdf = _sdf->GetElement("devel");
	      this->develPtr->checkFile = develSdf->Get<std::string>("file");
	      this->develPtr->from = develSdf->Get<int>("from");
	      this->develPtr->to = develSdf->Get<int>("to");
	    }
      }}
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::StartLimbX(XtendedActorPtr _actor, std::string _limb, std::string _arg)
    {
      std::string actorName = _actor->GetName();
      if (this->skelManager.find(actorName) == this->skelManager.end())
        {
	  std::cerr << "Unexpected actor for applying XTracker." << "\n";
          return;
        }

      if (_actor->SetLimbMotion(_limb, [=](XtendedActorPtr _a, std::string _s)
                                {return this->UpdateLimbX(_a, _s);}) == false)
        {
      	  std::cerr << "Unexpected limb called in XTrackerLimb" << "\n";
          return;
        }

      this->skelManager[actorName].userId = std::stoi(_arg);

      std::vector<XLimbSkeletonJoint> trackerJoints = this->skelManager[actorName].joints;
      std::vector<std::string> limbNodeList = _actor->GetLimbNodeList(_limb);
      this->skelManager[actorName].limb = _limb;

      for (unsigned int i = 0; i < trackerJoints.size(); ++i)
        {
          if (std::find(limbNodeList.begin(), limbNodeList.end(),
                        trackerJoints[i].nameInLimb) != limbNodeList.end())
            this->skelManager[actorName].joints[i].trackJoint = true;
          else
            this->skelManager[actorName].joints[i].trackJoint = false;
        }

      this->skelManager[actorName].next = _actor->xNull;
      this->skelManager[actorName].nextArg = "";


      {{
	  std::map<nite::JointType, TestSample> testSample0;
	  std::map<nite::JointType, TestSample> testSample1;
	  std::map<nite::JointType, TestSample> testSample2;

	  testSample0[nite::JOINT_TORSO] =
	    {-93.2252, -116.494, 1210.22, -0.0868898, 0.0291054, -0.00640775, 0.995772};
	  testSample0[nite::JOINT_NECK] =
	    {-91.9454, 47.1084, 1181.41, -0.372878, 0.0250541, -0.0506091, 0.92616};
	  testSample0[nite::JOINT_HEAD] =
	    {-72.6948, 230.946, 1003.62, -0.372878, 0.0250541, -0.0506091, 0.92616};
	  testSample0[nite::JOINT_LEFT_SHOULDER] =
	    {-232.268, 49.6133, 1189.4, 0.378131, 0.135814, 0.201702, 0.893246};
	  testSample0[nite::JOINT_LEFT_ELBOW] =
	    {-447.452, -63.3914, 1211.39, 0.362086, -0.709752, -0.22926, 0.559093};
	  testSample0[nite::JOINT_LEFT_HAND] =
	    {-413.259, 170.504, 1020.83, 0, 0, 0, 0};
	  testSample0[nite::JOINT_RIGHT_SHOULDER] =
	    {48.3769, 44.6035, 1173.42, 0.443912, -0.0888749, -0.297352, 0.84061};
	  testSample0[nite::JOINT_RIGHT_ELBOW] =
	    {243.011, -94.934, 1145.8, 0.476497, 0.689692, 0.241715, 0.488722};
	  testSample0[nite::JOINT_RIGHT_HAND] =
	    {223.146, 165.322, 1016.54, 0, 0, 0, 0};

	  testSample1[nite::JOINT_TORSO] =
	    {-96.4649, -167.518, 1206.38, -0.0445905, 0.0658256, 4.34068e-05, 0.996834};
	  testSample1[nite::JOINT_NECK] =
	    {-97.7449, 46.508, 1187.28, -0.197418, 0.062788, -0.0392031, 0.97752};
	  testSample1[nite::JOINT_HEAD] =
	    {-85.2139, 268.592, 1092.82, -0.197418, 0.062788, -0.0392031, 0.97752};
	  testSample1[nite::JOINT_LEFT_SHOULDER] =
	    {-234.595, 47.3064, 1205.4, -0.563023, -0.571578, -0.270683, 0.53201};
	  testSample1[nite::JOINT_LEFT_ELBOW] =
	    {-285.556, -43.2787, 972.836, 0.524071, 0.68714, 0.248003, -0.437817};
	  testSample1[nite::JOINT_LEFT_HAND] =
	    {-263.953, -204.68, 696.392, 0, 0, 0, 0};
	  testSample1[nite::JOINT_RIGHT_SHOULDER] =
	    {39.1049, 45.7095, 1169.16, 0.0320822, 0.389619, -0.45536, 0.799884};
	  testSample1[nite::JOINT_RIGHT_ELBOW] =
	    {108.888, -128.562, 1007.52, 0.201902, 0.663176, -0.409411, 0.593141};
	  testSample1[nite::JOINT_RIGHT_HAND] =
	    {36.4491, -202.027, 686.51, 0, 0, 0, 0};

	  testSample2[nite::JOINT_TORSO] =
	    {-73.8175, -102.027, 1262.87, -0.0739522, 0.0547072, 0.00506788, 0.995746};
	  testSample2[nite::JOINT_NECK] =
	    {-77.4157, 93.6737, 1233.84, -0.236559, 0.0517393, -0.00487111, 0.970226};
	  testSample2[nite::JOINT_HEAD] =
	    {-80.7593, 291.269, 1131.59, -0.236559, 0.0517393, -0.00487111, 0.970226};
	  testSample2[nite::JOINT_LEFT_SHOULDER] =
	    {-219.02, 93.3886, 1249.47, -0.0280406, 0.158467, 0.433712, 0.886564};
	  testSample2[nite::JOINT_LEFT_ELBOW] =
	    {-360.335, -93.8951, 1324.69, 0.0401935, 0.0178739, 0.426908, 0.903225};
	  testSample2[nite::JOINT_LEFT_HAND] =
	    {-573.529, -353.352, 1324.01, 0, 0, 0, 0};
	  testSample2[nite::JOINT_RIGHT_SHOULDER] =
	    {64.1882, 93.9588, 1218.21, 0.307874, 0.378595, -0.0576897, 0.870949};
	  testSample2[nite::JOINT_RIGHT_ELBOW] =
	    {239.666, 126.893, 1045.63, 0.339649, 0.566715, 0.0552672, 0.748611};
	  testSample2[nite::JOINT_RIGHT_HAND] =
	    {348.018, 271.044, 795.693, 0, 0, 0, 0};

	  if (this->develPtr->checkFile == "testSample")
	    {
	      std::vector<std::map<nite::JointType, TestSample> > samples =
		{testSample0, testSample1, testSample2};
	      this->develPtr->count = 0;
	      this->develPtr->samples.push_back(samples[this->develPtr->from]);
	      return;
	    }

	  std::string fileName =
	    std::string(getenv("HRISYS_LOG_PATH")) + "/" + this->develPtr->checkFile;

	  std::ifstream file(fileName);
	  if (file.fail())
	    {
	      std::cerr << "bad log file" << std::endl;
	      return;
	    }

	  int joint;
	  double x, y, z, qx, qy, qz, qw;
	  int line = -1;
	  std::map<nite::JointType, TestSample> sample;
	  this->develPtr->samples.reserve(this->develPtr->to - this->develPtr->from + 1);
	  while (file >> joint >> x >> y >> z >> qx >> qy >> qz >> qw)
	    {
	      if (joint < 0)
		{
		  line++;

		  if (!sample.empty())
		    {
		      this->develPtr->samples.push_back(sample);
		      sample.clear();
		    }
		}

	      if (line < this->develPtr->from)
		continue;

	      if (line > this->develPtr->to)
		break;

	      sample[static_cast<nite::JointType>(joint)] = {x, y, z, qx, qy, qz, qw};
	    }

	  file.close();
      }}

    }

    //////////////////////////////////////////////////
    void XTrackerLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      {{
	  std::string actorName = _actor->GetName();
	  this->skelManager[actorName].next
	    ->StartLimbX(_actor, _limb, this->skelManager[actorName].nextArg);
      }}
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      {{
	  std::map<std::string, XLimbActorSkeletonManager>::iterator it;

	  for (std::map<std::string, XLimbActorSkeletonManager>::iterator iter =
		 this->skelManager.begin(); iter != this->skelManager.end(); ++iter)
	    if ((iter->second).userId == 1)
	      {
		it = iter;
		break;
	      }

	  if (it == this->skelManager.end())
	    return;

	  XtendedActorPtr target =
	    boost::static_pointer_cast<XtendedActor>
	    (this->world->GetByName(it->first));

	  if (this->develPtr->count >= this->develPtr->samples.size())
	    {
	      if (target->GetName() == it->first)
	      	this->FinishLimbX(target, this->skelManager[it->first].limb);
	      return;
	    }

	  std::map<std::string, math::Matrix4> frame;

	  for (unsigned int j = 0; j < (it->second).joints.size(); ++j)
	    {
	      XLimbSkeletonJoint joint = (it->second).joints[j];

	      if (!joint.trackJoint)
		continue;

	      if (joint.type == J_STATIC)
		{
		  math::Matrix4 nitePose(math::Matrix4::IDENTITY);
		  nitePose.SetTranslate(joint.translate);
		  frame[joint.nameInLimb] =
		    joint.priorFixer * nitePose * joint.posteriorFixer;
		  continue;
		}

	      if (joint.type == J_TRANSLATE)
		{
		  continue;
		}

	      if (joint.type == J_ROTATE)
		{
		  math::Vector3 targetNitePos;
		  math::Vector3 childNitePos;

		  auto setTemplate = [&](std::map<nite::JointType, TestSample> testSample)
		    { targetNitePos = math::Vector3(-testSample[joint.nameInNite].x,
						    testSample[joint.nameInNite].y,
						    -testSample[joint.nameInNite].z);
		      childNitePos = math::Vector3(-testSample[joint.childInNite].x,
						   testSample[joint.childInNite].y,
						   -testSample[joint.childInNite].z); };

		  setTemplate(this->develPtr->samples[this->develPtr->count]);

		  math::Vector3 atT0 = joint.childTranslate;
		  math::Vector3 atTn = childNitePos - targetNitePos;

		  if (atT0 == math::Vector3(0, 0, 0) ||
		      atTn == math::Vector3(0, 0, 0))
		    continue;

		  math::Vector3 n = atT0.Cross(atTn);
		  double theta = asin(n.GetLength() /
				      (atT0.GetLength() * atTn.GetLength()));

		  math::Matrix4 nitePose
		    (math::Quaternion(n.Normalize(), theta).GetAsMatrix4());

		  nitePose.SetTranslate(joint.translate);
		  frame[joint.nameInLimb] =
		    joint.priorFixer * nitePose * joint.posteriorFixer;
		}
	    }

	  if (target->GetName() == it->first)
	    for (std::map<std::string, math::Matrix4>::iterator iter =
		   frame.begin(); iter != frame.end(); ++iter)
	      target->SetNodeTransform(iter->first, iter->second);

	  this->develPtr->count++;
      }}
    }

  }
}

#endif
