#ifdef HRISYS_HAVE_OPENNI

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <NiTE.h>

#include "gazebo/physics/World.hh"

#include "../utils/UncertainMemoryFilter.hh"
#include "../utils/GreedyDecisionFilter.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "XTrackerLimb.hh"

#define NUM_OF_NITE_JOINTS 22 /// 15 + 7 virtual joints

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    XTrackerLimb::XTrackerLimb(WorldPtr _world) : LimbXtentions(_world)
    {
    }

    //////////////////////////////////////////////////
    XTrackerLimb::~XTrackerLimb()
    {
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
      nite::NiTE::initialize();
      nite::Status niteRc = this->userTracker.create();

      if (niteRc != nite::STATUS_OK)
        {
	  std::cerr << "Creating tracker failed.\n";
          this->doTrack = false;
	  return;
        }

      this->doTrack = true;

      /// define the OpenNI skeleton nodes
      /// The below assumes that the actor model is constructed with
      /// only rotational joints.
      /// For models that are constructed in the OpenNI translation
      /// joint form, a similar but different scheme is required.

      /// create the OpenNI skeleton node trees

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

      /// lambda function is defined for code maintainability
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

      /// The root joint has no parent and differs from other joints.
      /// Note that the root rotation assumes Y_UP for actor model.
      /// NiTE is left-hand coordinate, but we expect it as Y_UP right-hand.
      /// Note that NiTE->actor coordinate conversion is done in update.
      /// Some of the joints will not use NiTE coordinate directely.
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

      /// setup the actors to be controlled (read from the SDF)

      std::stringstream ss(_sdf->Get<std::string>("actors"));
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> actors(begin, end);

      /// setup filters
      sdf::ElementPtr longFilterSdf;
      sdf::ElementPtr shortFilterSdf;
      bool hasLongFilter = false;
      bool hasShortFilter = false;
      if (_sdf->HasElement("filter0"))
	{
	  longFilterSdf = _sdf->GetElement("filter0");
	  hasLongFilter = true;
	}
      if (_sdf->HasElement("filter1"))
	{
	  shortFilterSdf = _sdf->GetElement("filter1");
	  hasShortFilter = true;
	}
      this->quadratic =
	[=](math::Vector3 v, math::Vector3 u){ return (v - u).GetLength(); };


      for (unsigned int i = 0; i < actors.size(); ++i)
        {
          XLimbActorSkeletonManager actorSkelManager;

          /// try to get actor
	  /// Note: There should be a forward check for name validity !
          XtendedActorPtr actor =
            boost::static_pointer_cast<XtendedActor>(this->world->GetByName(actors[i]));

          /// try to get skeleton map
	  std::map<std::string, std::string> skelMap = actor->GetSkelMap("mapOpenNI");
          if (skelMap.empty())
            {
	      std::cerr << actors[i] << " does not have mapOpenNI.\n";
              continue;
            }
          if (skelMap.size() != nodes.size())
            {
	      std::cerr << "mapOpenNI has illegal number of joints.\n";
              continue;
            }

          /// create a reverse skeleton map
	  std::map<std::string, std::string> rSkelMap;
          for (std::map<std::string, std::string>::iterator iter =
                 skelMap.begin(); iter != skelMap.end(); ++iter)
            {
              rSkelMap[iter->second] = iter->first;
            }

	  /// check validity and re-length nodes
          bool errorDetectedInCheck = false;
	  std::map<std::string, math::Matrix4> translationAligner;
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];
	      common::SkeletonNode *nodeInActor =
                actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()]);

              if (!node || !nodeInActor)
                {
                  /// fail : expected joint does not exist
		  std::cerr << "mapOpenNI has illegal joint name : "
			    << node->GetName() << " or "
			    << nodeInActor->GetName() << "\n";
                  errorDetectedInCheck = true;
                  break;
                }

              if (node->GetChildCount() != nodeInActor->GetChildCount())
                  /// fail : child number differs, thus joints aren't compatible
		  /// exception : if node is an end joint, then childs may differ
		  if (node->GetChildCount() != 0)
		    {
		      std::cerr << "mapOpenNI has illegal joint pair : "
				<< node->GetName() << " and "
				<< nodeInActor->GetName() << "\n";
		      errorDetectedInCheck = true;
		      break;
		    }

	      /// scale nite offset to actor link length
	      math::Matrix4 transform(node->GetTransform());
	      math::Vector3 niteOffset = node->GetTransform().GetTranslation();
	      math::Vector3 daeOffset = nodeInActor->GetTransform().GetTranslation();
              transform.SetTranslate(daeOffset.GetLength() * niteOffset.Normalize());
              node->SetTransform(transform, false);
	    }

	  /// if there was an error, try the next actor
          if (errorDetectedInCheck)
            continue;

          /// calculate the translation align matrix
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];
	      common::SkeletonNode *nodeInActor =
                actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()]);

              if (node->GetParent() != NULL)
                {
                  if (node->GetParent()->GetChildCount() > 1)
                    {
                      /// parent link has multiple child links
                      if (node->GetTransform().GetTranslation() ==
                          math::Vector3(0, 0, 0))
                        /// parent link is a virtual link (has no length)
                        translationAligner[node->GetName()] = math::Matrix4::IDENTITY;
                    }
                }

              if (node->GetChildCount() == 0)
                /// if this is an end bone, then link i matrix is already calculated
                continue;

              if (node->GetName() ==
                  skelMap[actor->GetSkeletonData()->GetRootNode()->GetName()])
                {
                  /// if this is root, then some setup is needed to match bvh and dae
                  translationAligner[node->GetName()] =
                    actor->GetSkeletonData()->GetRootNode()
                    ->GetTransform().GetRotation().GetAsMatrix4();
		  math::Matrix4 tmp = translationAligner[node->GetName()];
                  tmp.SetTranslate(node->GetTransform().GetTranslation());
                  node->SetTransform(tmp, true);
                }

              if (node->GetChildCount() > 1)
                /// if link i has multiple childs
                continue;

              /// else, which means link i only has a single child link i+1

	      /// get link i+1 posture direction in world coordinates
	      math::Vector3 relativeBVH =
                node->GetChild(0)->GetModelTransform().GetTranslation()
                - node->GetModelTransform().GetTranslation();
	      math::Vector3 relativeDAE =
                nodeInActor->GetChild(0)->GetModelTransform().GetTranslation()
                - nodeInActor->GetModelTransform().GetTranslation();

              if (relativeDAE == math::Vector3(0, 0, 0))
                {
                  /// unexpected
		  std::cerr << "WARNING! Weird joints in model?" << std::endl;
                  continue;
                }

              /// calculate world coordinate rotation quaternion
	      math::Vector3 n = relativeBVH.Cross(relativeDAE);
              double theta =
                asin(n.GetLength() / (relativeDAE.GetLength() * relativeBVH.GetLength()));

              /// calculate nite to dae of link i+1
              translationAligner[node->GetChild(0)->GetName()]
                = nodeInActor->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();

              /// fix nite posture of all links until link i
	      math::Matrix4 tmp =
                node->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();
              tmp.SetTranslate(node->GetTransform().GetTranslation());
              node->SetTransform(tmp, true);
	    }

	  /// calculate the rotation align matrix
	  std::map<std::string, math::Matrix4> rotationAligner;
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];

              if (node->GetName()
		  == skelMap[actor->GetSkeletonData()->GetRootNode()->GetName()])
                {
                  /// rotation should not be aligned with root
                  rotationAligner[node->GetName()] = math::Matrix4::IDENTITY;
                  continue;
                }

              /// in case an aligner was not correctly calculated, set a value to prevent nan
              if (translationAligner[node->GetName()] == math::Matrix4::ZERO)
                translationAligner[node->GetName()] = math::Matrix4::IDENTITY;

              rotationAligner[node->GetName()]
                = node->GetTransform().GetRotation().GetAsMatrix4().Inverse()
                * translationAligner[node->GetName()].Inverse()
                * actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()])
                ->GetTransform().GetRotation().GetAsMatrix4();
            }

          /// setup the joints
          actorSkelManager.joints.resize(NUM_OF_NITE_JOINTS);
          for (std::map<std::string, XLimbTemplateJoint>::iterator iter =
                 templateSkelJoints.begin();
               iter != templateSkelJoints.end(); ++iter)
            {
	      common::SkeletonNode* thisJoint = nodes[(iter->second).nodeId];
	      common::SkeletonNode* parentJoint = thisJoint->GetParent();
	      common::SkeletonNode* childJoint = NULL;

	      /// search for nite parent
	      while (parentJoint != NULL)
		{
		  if ((iter->second).nameInNite !=
		      templateSkelJoints[parentJoint->GetName()].nameInNite)
		    break;
		  parentJoint = parentJoint->GetParent();
		}
	      if (parentJoint == NULL)
		/// occasions such as root
		parentJoint = thisJoint;

	      /// search for nite child
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
	      /// filters
	      if (hasLongFilter)
		{
		  joint.longFilter = UncertainMemoryFilterPtr<math::Vector3>
		    (new UncertainMemoryFilter<math::Vector3>(longFilterSdf));
		  joint.longFilter->SetCostFunction(this->quadratic);
		}
	      if (hasShortFilter)
		{
		  joint.shortFilter = GreedyDecisionFilterPtr<math::Vector3>
		    (new GreedyDecisionFilter<math::Vector3>(shortFilterSdf));
		  joint.shortFilter->SetCostFunction(this->quadratic);
		}

              actorSkelManager.joints[(iter->second).nodeId] = joint;
            }

	  /// filters
	  if (hasLongFilter)
	    actorSkelManager.longFilterOn = true;
	  if (hasShortFilter)
	    actorSkelManager.shortFilterOn = true;

          /// save the fix-matrix
          this->skelManager[actors[i]] = actorSkelManager;
	}

      /// initialize onRegister
      this->onRegister = 0;
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::StartLimbX(XtendedActorPtr _actor, std::string _limb, std::string _arg)
    {
      if (!doTrack)
        {
	  std::cerr << "XTracker aborted due to initialization failure.\n";
          return;
        }

      std::string actorName = _actor->GetName();
      if (this->skelManager.find(actorName) == this->skelManager.end())
        {
	  std::cerr << "Unexpected actor for applying XTracker.\n";
          return;
        }

      if (_actor->SetLimbMotion(_limb, [=](XtendedActorPtr _a, std::string _s)
                                {return this->UpdateLimbX(_a, _s);}) == false)
        {
	  std::cerr << "Unexpected limb called in XTrackerLimb.\n";
          return;
        }

      /// the number of actors that register update increases
      this->skelManager[actorName].registered = true;
      this->onRegister++;

      /// set the headActor if first actor to activate XTracker
      if (this->onRegister == 1)
        this->headActor = actorName;

      /// set the user to track
      this->skelManager[actorName].userId = std::stoi(_arg);

      std::vector<XLimbSkeletonJoint> trackerJoints = this->skelManager[actorName].joints;
      std::vector<std::string> limbNodeList = _actor->GetLimbNodeList(_limb);
      this->skelManager[actorName].limb = _limb;

      /// set the joints to track
      for (unsigned int i = 0; i < trackerJoints.size(); ++i)
        {
          if (std::find(limbNodeList.begin(), limbNodeList.end(),
                        trackerJoints[i].nameInLimb) != limbNodeList.end())
	    /// joint is in limbNodeList, so track the joint
            this->skelManager[actorName].joints[i].trackJoint = true;
          else
            this->skelManager[actorName].joints[i].trackJoint = false;
        }

      this->skelManager[actorName].next = _actor->xNull;
      this->skelManager[actorName].nextArg = "";
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      std::string actorName = _actor->GetName();
      this->skelManager[actorName].registered = false;
      this->onRegister--;

      std::cerr << "Finishing Tracker " << _actor->GetName() << "\n";

      /// When some actors are still tracked, and this is head actor
      /// switch headActor to one of the tracked actors
      if ((onRegister > 0) && (actorName == headActor))
        for (std::map<std::string, XLimbActorSkeletonManager>::iterator it =
               this->skelManager.begin(); it != this->skelManager.end(); ++it)
          if ((it->second).registered == true)
            {
              headActor = it->first;
              break;
            }

      this->skelManager[actorName].next
        ->StartLimbX(_actor, _limb, this->skelManager[actorName].nextArg);
    }

    //////////////////////////////////////////////////
    void XTrackerLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      /// If multiple actors register the UpdateLimbX,
      /// make sure the Update is called only once.
      if (_actor->GetName() != headActor)
        return;

      nite::Status niteRc = this->userTracker.readFrame(&this->userTrackerFrame);

      if (niteRc != nite::STATUS_OK)
        {
          return;
        }

      const nite::Array<nite::UserData>& users = this->userTrackerFrame.getUsers();
      for (unsigned int i = 0; i < users.getSize(); ++i)
        {
          const nite::UserData& user = users[i];
          if (user.isNew())
            {
              /// only track if the id is expected
              for (std::map<std::string, XLimbActorSkeletonManager>::iterator it =
                     this->skelManager.begin(); it != this->skelManager.end(); ++it)
                if ((it->second).userId == user.getId())
                  {
                    this->userTracker.startSkeletonTracking(user.getId());
                    break;
                  }
              continue;
            }

	  std::map<std::string, XLimbActorSkeletonManager>::iterator it;

	  for (std::map<std::string, XLimbActorSkeletonManager>::iterator iter =
		 this->skelManager.begin(); iter != this->skelManager.end(); ++iter)
	    if ((iter->second).userId == user.getId())
	      {
		it = iter;
		break;
	      }

	  if (user.isLost())
	    {
	      XtendedActorPtr target =
		boost::static_pointer_cast<XtendedActor>
		(this->world->GetByName(it->first));

	      if (target->GetName() == it->first)
		this->FinishLimbX(target, this->skelManager[it->first].limb);
	      continue;
	    }

	  /// userId matches

	  if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
            {
	      std::map<std::string, math::Matrix4> frame;

	      /// filter beforehand
	      std::map<nite::JointType, math::Vector3> filteredJoint;
	      for (unsigned int j = 0; j < (it->second).joints.size(); ++j)
		{
		  XLimbSkeletonJoint joint = (it->second).joints[j];

		  if (!joint.trackJoint)
		    continue;

		  nite::SkeletonJoint targetJoint =
		    user.getSkeleton().getJoint(joint.nameInNite);

		  /// Only add to filter when there is confidence.
		  if (targetJoint.getPositionConfidence() < 0.5f)
		    continue;

		  /// Nite is Z_REVERSE, X_REVERSE, left->right coordinate.
		  math::Vector3 filteredPos (-targetJoint.getPosition().x,
					     targetJoint.getPosition().y,
					     -targetJoint.getPosition().z);

		  if ((it->second).longFilterOn)
		    {
		      (it->second).joints[j].longFilter->AddData(filteredPos);
		      filteredPos = (it->second).joints[j].longFilter->GetData();
		    }

		  if ((it->second).shortFilterOn)
		    {
		      (it->second).joints[j].shortFilter->AddData(filteredPos);
		      filteredPos = (it->second).joints[j].shortFilter->GetData();
		    }

		  filteredJoint[joint.nameInNite] = filteredPos;
		}

	      std::map<nite::JointType, math::Quaternion> niteFrame;
              for (unsigned int j = 0; j < (it->second).joints.size(); ++j)
                {
                  XLimbSkeletonJoint joint = (it->second).joints[j];

                  if (!joint.trackJoint)
                    /// joint is not a tracked joint
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
		      /// -------------------------------------------------
                      /// -------------------- TODO -----------------------
                      /// -------------------------------------------------
		      continue;
		    }

		  if (joint.type == J_ROTATE)
		    {
		      nite::SkeletonJoint targetJoint =
			user.getSkeleton().getJoint(joint.nameInNite);
		      nite::SkeletonJoint childJoint =
			user.getSkeleton().getJoint(joint.childInNite);

		      /// The orientation is calculated from joint positions.
		      /// Only calculate when there is confidence.
		      if ((targetJoint.getPositionConfidence() < 0.5f) ||
			  (childJoint.getPositionConfidence() < 0.5f))
			continue;

		      math::Vector3 targetNitePos = filteredJoint[joint.nameInNite];
		      math::Vector3 childNitePos = filteredJoint[joint.childInNite];

		      math::Quaternion parentNiteRot (0, 0, 0);
		      auto it = niteFrame.find(joint.parentInNite);
		      if (it != niteFrame.end())
			parentNiteRot = niteFrame[joint.parentInNite];

		      math::Vector3 atT0 = parentNiteRot * joint.childTranslate;
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

		      niteFrame[joint.nameInNite] =
			nitePose.GetRotation() * parentNiteRot;
		    }
		} /// for j in joints

	      XtendedActorPtr target =
		boost::static_pointer_cast<XtendedActor>
		(this->world->GetByName(it->first));

	      /// send the current orientation to actor
	      if (target->GetName() == it->first)
		for (std::map<std::string, math::Matrix4>::iterator iter =
		       frame.begin(); iter != frame.end(); ++iter)
		  target->SetNodeTransform(iter->first, iter->second);
            } /// if nite::SKELETON_TRACKED

        } /// for i in users
    } /// UpdateLimbX

  }
}

#endif
