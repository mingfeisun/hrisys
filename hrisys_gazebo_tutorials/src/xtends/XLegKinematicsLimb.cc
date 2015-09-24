#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "gazebo/physics/World.hh"

#include "../XtendedActor.hh"
#include "../LimbXtentions.hh"
#include "XLegKinematicsLimb.hh"

#define NUM_OF_LEG_JOINTS 11

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    XLegKinematicsLimb::XLegKinematicsLimb(WorldPtr _world) : LimbXtentions(_world)
    {
      this->getFromXTracker = false;
    }

    //////////////////////////////////////////////////
    XLegKinematicsLimb::~XLegKinematicsLimb()
    {
    }

    //////////////////////////////////////////////////
    void XLegKinematicsLimb::InitLimbX(sdf::ElementPtr _sdf)
    {
      unsigned int nodeIdCount = 0;
      std::vector<common::SkeletonNode*> nodes;
      nodes.resize(NUM_OF_LEG_JOINTS);
      std::map<std::string, unsigned int> nodeIds;

      /// lambda function is defined for code maintainability
      auto setNewJoint = [&](std::string name, std::string parent,
			     double x, double y, double z)
        { ++nodeIdCount;
	  nodes[nodeIdCount] =
          new common::SkeletonNode(nodes[nodeIds[parent]],
                                   name, name, common::SkeletonNode::JOINT);
	  nodes[nodeIdCount]
          ->SetTransform((math::Matrix4(1, 0, 0, x,
                                        0, 1, 0, y,
                                        0, 0, 1, z,
                                        0, 0, 0, 1)), false);
	  nodeIds[name] = nodeIdCount;};

      nodes[0] =
	new common::SkeletonNode(NULL, "ROOT", "ROOT",
				 common::SkeletonNode::JOINT);
      nodes[0]->SetTransform(math::Matrix4::IDENTITY, false);
      nodeIds["ROOT"] = 0;
      setNewJoint("VIRTUAL_LEFT_HIP", "ROOT", 0, 0, 0);
      setNewJoint("LEFT_HIP", "VIRTUAL_LEFT_HIP", 0.707, -0.707, 0);
      setNewJoint("LEFT_KNEE", "LEFT_HIP", 0, -1, 0);
      setNewJoint("LEFT_FOOT", "LEFT_KNEE", 0, -1, 0);
      setNewJoint("LEFT_TOE", "LEFT_FOOT", 0, 0, 1);
      setNewJoint("VIRTUAL_RIGHT_HIP", "ROOT", 0, 0, 0);
      setNewJoint("RIGHT_HIP", "VIRTUAL_RIGHT_HIP", -0.707, -0.707, 0);
      setNewJoint("RIGHT_KNEE", "RIGHT_HIP", 0, -1, 0);
      setNewJoint("RIGHT_FOOT", "RIGHT_KNEE", 0, -1, 0);
      setNewJoint("RIGHT_TOE", "RIGHT_FOOT", 0, 0, 1);

      std::stringstream ssActors(_sdf->Get<std::string>("actors"));
      std::istream_iterator<std::string> beginActors(ssActors);
      std::istream_iterator<std::string> endActors;
      std::vector<std::string> actors(beginActors, endActors);

      std::stringstream ssMaps(_sdf->Get<std::string>("maps"));
      std::istream_iterator<std::string> beginMaps(ssMaps);
      std::istream_iterator<std::string> endMaps;
      std::vector<std::string> maps(beginMaps, endMaps);

      if (actors.size() != maps.size())
	{
	  std::cerr << "Number of defined maps does not match, XLegKinematics.\n";
	  return;
	}

      for (unsigned int i = 0; i < actors.size(); ++i)
        {
          XLimbLegSkeletonManager legSkelManager;

          /// try to get actor
	  /// Note: There should be a forward check for name validity !
          XtendedActorPtr actor =
            boost::static_pointer_cast<XtendedActor>(this->world->GetByName(actors[i]));

	  /// try to get skeleton map
	  std::map<std::string, std::string> skelMap = actor->GetSkelMap(maps[i]);
          if (skelMap.empty())
            {
	      std::cerr << actors[i] << " does not have" << maps[i] << ".\n";
              continue;
            }

	  /// create a reverse skeleton map
	  std::map<std::string, std::string> rSkelMap;
          for (auto iter = skelMap.begin(); iter != skelMap.end(); ++iter)
	    rSkelMap[iter->second] = iter->first;

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
		  std::cerr << maps[i] << " does not have joint : "
			    << node->GetName() << " or has illegal : "
			    << nodeInActor->GetName() << "\n";
                  errorDetectedInCheck = true;
                  break;
                }

              if (node->GetChildCount() != nodeInActor->GetChildCount())
		/// fail : child number differs, thus joints aren't compatible
		/// exception1 : if node is an end joint, then childs may differ
		/// exception2 : if node is a root joint, then childs may differ
		if (node->GetChildCount() != 0 && node->GetName() != "ROOT")
		  {
		    std::cerr << maps[i] << " has illegal joint pair : "
			      << node->GetName() << " and "
			      << nodeInActor->GetName() << "\n";
		    errorDetectedInCheck = true;
		    break;
		  }

	      /// scale nite offset to actor link length
	      math::Matrix4 transform(node->GetTransform());
	      math::Vector3 legOffset = node->GetTransform().GetTranslation();
	      math::Vector3 daeOffset = nodeInActor->GetTransform().GetTranslation();
              transform.SetTranslate(daeOffset.GetLength() * legOffset.Normalize());
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

              /// calculate leg to dae of link i+1
              translationAligner[node->GetChild(0)->GetName()]
                = nodeInActor->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();

              /// fix leg posture of all links until link i
	      math::Matrix4 tmp =
                node->GetModelTransform().GetRotation().GetAsMatrix4().Inverse()
                * math::Quaternion(n.Normalize(), theta).GetAsMatrix4()
                * node->GetModelTransform().GetRotation().GetAsMatrix4();
              tmp.SetTranslate(node->GetTransform().GetTranslation());
              node->SetTransform(tmp, true);
	    }

	  /// calculate the rotation align matrix and setup the joints
	  std::map<std::string, math::Matrix4> rotationAligner;
          legSkelManager.joints.resize(NUM_OF_LEG_JOINTS);
	  legSkelManager.links.resize(3);
          for (unsigned int j = 0; j < nodes.size(); j++)
            {
	      common::SkeletonNode *node = nodes[j];

              if (node->GetName()
		  == skelMap[actor->GetSkeletonData()->GetRootNode()->GetName()])
                {
                  /// rotation should not be aligned with root
                  rotationAligner[node->GetName()] = math::Matrix4::IDENTITY;
                }
	      else
		{
		  /// in case an aligner was not correctly calculated
		  /// set a value to prevent nan
		  if (translationAligner[node->GetName()] == math::Matrix4::ZERO)
		    translationAligner[node->GetName()] = math::Matrix4::IDENTITY;

		  rotationAligner[node->GetName()]
		    = node->GetTransform().GetRotation().GetAsMatrix4().Inverse()
		    * translationAligner[node->GetName()].Inverse()
		    * actor->GetSkeletonData()->GetNodeByName(rSkelMap[node->GetName()])
		    ->GetTransform().GetRotation().GetAsMatrix4();
		}

              XLimbLegSkeletonJoint joint;
              joint.nameInLimb = rSkelMap[node->GetName()];
	      joint.translate = node->GetTransform().GetTranslation();
              joint.priorFixer = translationAligner[node->GetName()];
              joint.posteriorFixer = rotationAligner[node->GetName()];
	      joint.calculateOnly = false;

              legSkelManager.joints[j] = joint;
	      legSkelManager.rootOffset = math::Vector3(0, 0, 0);

	      if (node->GetName() == "LEFT_TOE")
		legSkelManager.links[0] = joint.translate.GetLength();
	      if (node->GetName() == "LEFT_FOOT")
		legSkelManager.links[1] = joint.translate.GetLength();
	      if (node->GetName() == "LEFT_KNEE")
		legSkelManager.links[2] = joint.translate.GetLength();
            }

	  /// save the fix-matrix
          this->skelManager[actors[i]] = legSkelManager;
	}
    }

    //////////////////////////////////////////////////
    void XLegKinematicsLimb::StartLimbX(XtendedActorPtr _actor,
					std::string _limb, std::string _arg)
    {
      std::string actorName = _actor->GetName();
      if (this->skelManager.find(actorName) == this->skelManager.end())
        {
	  std::cerr << "Unexpected actor for applying XLegKinematics.\n";
          return;
        }

      bool calculateOnly = false;
      if (_limb.find("~~") != std::string::npos)
	{
	  _limb.erase(0, 2); /// erase ~~(expected in head) from limb name
          if (_actor->GetLimbNodeList(_limb).empty())
            {
	      std::cerr << "Unexpected calc-only limb called in XLegKinematics.\n";
              return;
            }
          calculateOnly = true;
	}
      else if (_actor->SetLimbMotion(_limb, [=](XtendedActorPtr _a, std::string _s)
				     {return this->UpdateLimbX(_a, _s);}) == false)
        {
	  std::cerr << "Unexpected limb called in XLegKinematics.\n";
          return;
        }

      /// set the joints to calculateOnly
      for (unsigned int i = 0; i < this->skelManager[actorName].joints.size(); ++i)
	this->skelManager[actorName].joints[i].calculateOnly = calculateOnly;

      this->skelManager[actorName].next = _actor->xNull;
      this->skelManager[actorName].nextArg = "";
    }

    //////////////////////////////////////////////////
    void XLegKinematicsLimb::FinishLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      std::string actorName = _actor->GetName();
      this->skelManager[actorName].next
        ->StartLimbX(_actor, _limb, this->skelManager[actorName].nextArg);
    }

# ifdef HRISYS_HAVE_OPENNI
    //////////////////////////////////////////////////
    void XLegKinematicsLimb::SetFromTracker(XTrackerLimbPtr _tracker)
    {
      std::cout << "XLegKinematics linked with XTracker.\n";
      this->fromTracker = _tracker;
      this->getFromXTracker = true;
    }
# endif

    //////////////////////////////////////////////////
    void XLegKinematicsLimb::SetRootOffset(std::string _name, math::Vector3 _reference)
    {
      auto it = this->skelManager.find(_name);
      if (it == this->skelManager.end())
	{
	  std::cerr << "Root offset failed, XLegKinematics.\n";
	  return;
	}
      (it->second).rootOffset =
	_reference - math::Vector3(0, 0, (it->second).links[1] + (it->second).links[2]);
    }

    //////////////////////////////////////////////////
    std::map<std::string, math::Matrix4>
    XLegKinematicsLimb::GetCalculatedFrameData(std::string _name) const
    {
      auto it = this->skelManager.find(_name);
      if (it == this->skelManager.end())
	return std::map<std::string, math::Matrix4>();
      return (it->second).legFrame;
    }

    //////////////////////////////////////////////////
    void XLegKinematicsLimb::UpdateLimbX(XtendedActorPtr _actor, std::string _limb)
    {
      XLimbLegSkeletonManager leg = this->skelManager[_actor->GetName()];
      math::Vector3 rootPosition(0, 0, leg.links[1] + leg.links[2]);
      math::Matrix4 rootTransform;
      XLimbLegSkeletonJoint rootJoint = leg.joints[0];

      if (this->getFromXTracker)
	{
	  /// there should be a manual call to tracker when no limb calls update

	  auto frameFromTracker =
	    this->fromTracker->GetTrackedFrameData(_actor->GetName());
	  auto it = frameFromTracker.find(rootJoint.nameInLimb);
	  if (it == frameFromTracker.end())
	    return;
	  rootTransform = (it->second);
	  rootPosition += rootTransform.GetTranslation();
	  rootTransform.SetTranslate(math::Vector3(rootPosition.x + leg.rootOffset.x,
						   leg.rootOffset.y,
						   rootPosition.z + leg.rootOffset.z));
	}
      else
	{
	  rootTransform = _actor->GetSkeletonData()->GetRootNode()->GetTransform();
	  rootPosition += rootTransform.GetTranslation();
	  rootTransform.SetTranslate(rootPosition + leg.rootOffset);
	}

      /// the following calculates a 2D 3-link leg model :
      /// end : pos = {0, leg.links[1] + leg.links[2]}
      /// upper-leg-link : theta = 0.0
      /// knee : pos = {0, leg.links[1]}
      /// foreleg-link : theta = 0.0
      /// ankle : pos = {0, 0}
      /// foot-link : theta = 0.0
      /// toe : pos= {leg.links[0], 0}

      /// end position from ankle
      math::Vector2d endPosition(rootPosition.x, rootPosition.z);
      double toEndPosition =
	endPosition.x * endPosition.x + endPosition.y * endPosition.y;
      double maxLegPosition = leg.links[1] + leg.links[2];

      double kneeTheta, ankleTheta, toeTheta;
      if (toEndPosition >= maxLegPosition * maxLegPosition)
	{
	  /// use toe only when knee and ankle cannot solve ik
	  kneeTheta = 0.0;
	  double legLength = maxLegPosition;
	  maxLegPosition += leg.links[0];
	  /// recalculate end position from toe origin
	  toEndPosition =
	    (endPosition.x - leg.links[0]) * (endPosition.x - leg.links[0]) +
	    endPosition.y * endPosition.y;
	  if (toEndPosition >= maxLegPosition * maxLegPosition)
	    {
	      /// ik cannot be solved
	      ankleTheta = M_PI * 0.25;
	      toeTheta = M_PI * 0.25;
	    }
	  else
	    {
	      ankleTheta = acos((toEndPosition -
				 leg.links[0] * leg.links[0] -
				 legLength * legLength) /
				(2 * leg.links[0] * legLength));
	      if (ankleTheta > 0) ankleTheta = -ankleTheta;
	      double k1 = leg.links[0] + legLength * cos(ankleTheta);
	      double k2 = legLength * sin(ankleTheta);
	      double gamma = M_PI / 2;
	      if (k1 > 0) gamma = atan(k2 / k1);
	      else if (k1 < 0) gamma = atan(k2 / k1) + M_PI;
	      toeTheta =
		acos((endPosition.x - leg.links[0]) / sqrt(k1 * k1 + k2 * k2)) - gamma;
	      toeTheta -= M_PI; /// heel on ground as angle 0.0
	      ankleTheta = M_PI * 0.5 + ankleTheta;
	      /// ankle perpendicular to foot position as angle 0.0
	    }
	}
      else
	{
	  /// solve ik by only using knee and ankle
	  kneeTheta = acos((toEndPosition -
			    leg.links[1] * leg.links[1] -
			    leg.links[2] * leg.links[2]) /
			   (2 * leg.links[1] * leg.links[2]));
	  if (kneeTheta < 0) kneeTheta = -kneeTheta;
	  double k1 = leg.links[1] + leg.links[2] * cos(kneeTheta);
	  double k2 = leg.links[2] * sin(kneeTheta);
	  double gamma = M_PI / 2;
	  if (k1 > 0) gamma = atan(k2 / k1);
	  else if (k1 < 0) gamma = atan(k2 / k1) + M_PI;
	  ankleTheta =
	    acos(endPosition.x / sqrt(k1 * k1 + k2 * k2)) - gamma;
	  ankleTheta -= M_PI * 0.5;
	  toeTheta = 0.0;
	}

      /// angles should not exceed human capabilities
      if (kneeTheta > M_PI * 0.75) kneeTheta = M_PI * 0.75;

      if (ankleTheta > M_PI * 0.25) ankleTheta = M_PI * 0.25;
      else if (ankleTheta < -M_PI * 0.78) ankleTheta = -M_PI * 0.78;

      if (toeTheta < -M_PI * 0.25) toeTheta = -M_PI * 0.25;
      else if (toeTheta > 0.0) toeTheta = 0.0;

      /// calculate relative rotation Matrix from initial position
      math::Matrix4 hipPose
	(math::Quaternion(math::Vector3(-1, 0, 0),
			  kneeTheta + ankleTheta - toeTheta).GetAsMatrix4());
      math::Matrix4 kneePose
	(math::Quaternion(math::Vector3(1, 0, 0), kneeTheta).GetAsMatrix4());
      math::Matrix4 toePose
	(math::Quaternion(math::Vector3(1, 0, 0), ankleTheta).GetAsMatrix4());

      std::vector<math::Matrix4> rotationMatrices =
	{math::Matrix4::IDENTITY, hipPose, kneePose, toePose, math::Matrix4::IDENTITY,
	 math::Matrix4::IDENTITY, hipPose, kneePose, toePose, math::Matrix4::IDENTITY,};

      /// send current frame

      /// root joint
      this->skelManager[_actor->GetName()].legFrame[rootJoint.nameInLimb] = rootTransform;
      if (!rootJoint.calculateOnly)
	_actor->SetNodeTransform(rootJoint.nameInLimb, rootTransform);

      /// for other joints
      for (unsigned int i = 1; i < NUM_OF_LEG_JOINTS; ++i) /// skip root
	{
	  XLimbLegSkeletonJoint joint = leg.joints[i];

	  math::Matrix4 pose = rotationMatrices[i-1];
	  pose.SetTranslate(joint.translate);
	  math::Matrix4 frame = joint.priorFixer * pose * joint.posteriorFixer;
	  this->skelManager[_actor->GetName()].legFrame[joint.nameInLimb] = frame;

	  if (!joint.calculateOnly)
	    _actor->SetNodeTransform(joint.nameInLimb, frame);
	}
    }
  }
}
