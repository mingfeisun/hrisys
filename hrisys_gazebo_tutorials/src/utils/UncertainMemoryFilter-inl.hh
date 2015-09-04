#ifndef _HRI_UNCERTAIN_MEMORY_FILTER_INL_H_
#define _HRI_UNCERTAIN_MEMORY_FILTER_INL_H_

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "UncertainMemoryFilter.hh"

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    template <typename T>
    UncertainMemoryFilter<T>::UncertainMemoryFilter(sdf::ElementPtr _sdf) :
      param {_sdf->Get<int>("observations"), _sdf->Get<int>("rejectSize"),
	_sdf->Get<int>("memorySize"),
	_sdf->Get<int>("uncertaintyThre"), _sdf->Get<float>("clusterDist")}
    {
      this->dominantGroup = 0;
      this->state = {0, 0};
    }

    //////////////////////////////////////////////////
    template <typename T>
    UncertainMemoryFilter<T>::~UncertainMemoryFilter()
    {
    }

    //////////////////////////////////////////////////
    template <typename T>
    void UncertainMemoryFilter<T>::SetCostFunction(std::function<double(T, T)> _fPtr)
    {
      this->cost = _fPtr;
    }

    //////////////////////////////////////////////////
    template <typename T>
    void UncertainMemoryFilter<T>::AddData(T _newData)
    {
      /// If deep memory capacity is full, select data to stash.
      if (this->deepMemory.size() >= this->param.memorySize)
	  this->UpdateMemory();

      /// Remove oldest observation
      if (deepMemory.size() >= this->param.observations)
	this->groups[groupVectorMap[deepMemory.size() - this->param.observations]]
	  .elementsInObservation--;

      /// Find group to insert the new data.
      int classification = -1;
      for (unsigned int i = 0; i < this->groups.size(); ++i)
	if (this->cost(_newData, this->groups[i].lastElementData) <
	    this->param.clusterDistance)
	  {
	    classification = this->groups[i].groupId;
	    this->groups[i].numberOfElements++;
	    this->groups[i].elementsInObservation++;
	    this->groups[i].lastElementData = _newData;
	    /// Get dominant group.
	    if (this->groups[i].elementsInObservation > this->groups[0].elementsInObservation)
	      {
		dominantGroup = this->groups[i].groupId;
		/// swap dominant group to head.
		dataGroup g = this->groups[0];
		this->groups[0] = this->groups[i];
		this->groupVectorMap[g.groupId] = i;
		this->groups[i] = g;
		this->groupVectorMap[this->dominantGroup] = 0;
	      }
	    break;
	  }

      /// If does not fit any group, create new group.
      if (classification < 0)
	{
	  dataGroup g;
	  g.groupId = this->state.nextGroupId;
	  g.numberOfElements = 1;
	  g.elementsInObservation = 1;
	  g.lastElementData = _newData;
	  this->groupVectorMap[g.groupId] = groups.size();
	  this->groups.push_back(g);
	  classification = this->state.nextGroupId;
	  this->state.nextGroupId++;
	}

      /// Add data to deepMemory.
      this->deepMemory.push_back(classification);

      /// If surgery point, add to surgery point list.
      if (classification != this->state.previousClass)
	{
	  surgeryPoint p;
	  p.pointIndex = this->deepMemory.size() - 1;
	  p.numberOfElements = 1;

	  int surgeryStartPoint = p.pointIndex - this->param.rejectSize;
	  if (surgeryStartPoint < 0)
	    surgeryStartPoint = 0;

	  for (unsigned int i = 0; i < this->points.size(); ++i)
	    if (this->points[i].pointIndex > surgeryStartPoint)
	      p.numberOfElements++;

	  this->points.push_back(p);
	}
      this->state.previousClass = classification;
    }


    //////////////////////////////////////////////////
    template <typename T>
    void UncertainMemoryFilter<T>::UpdateMemory()
    {
      int maxUncertaintyCount = 0;
      int uncertainStartPoint = 0;

      /// Find the most oldest point with most uncertain region.
      for (unsigned int i = 0; i < this->points.size(); ++i)
	if (this->points[i].numberOfElements > maxUncertaintyCount)
	  {
	    maxUncertaintyCount = this->points[i].numberOfElements;
	    uncertainStartPoint = this->points[i].pointIndex - this->param.rejectSize;
	  }

      if ((maxUncertaintyCount < this->param.uncertaintyThreshold) ||
	  (uncertainStartPoint < 0))
	  /// Recent datas are certain, so reject from the oldest data.
	  uncertainStartPoint = 0;
      /// Else, there are uncertain datas in some of the recent ones.

      /// Update memory.
      int j = 0;
      for (unsigned int i = 0; i < this->deepMemory.size(); ++i, ++j)
	{
	  while (i < uncertainStartPoint)
	    {
	      ++i;
	      ++j;
	    }

	  while (i < uncertainStartPoint + this->param.rejectSize)
	    {
	      /// Delete elements in rejected region from group.
	      this->groups[this->groupVectorMap[this->deepMemory[i]]]
		.numberOfElements--;
	      ++i;
	    }
	  if (i >= this->deepMemory.size()) break;

	  this->deepMemory[j] = this->deepMemory[i];
	}
      // this->deepMemory.resize(this->param.observations);
      this->deepMemory.resize(j);

      /// Modify surgery points.
      j = 0;
      bool surgery = true;
      for (unsigned int i = 0; i < this->points.size(); ++i, ++j)
	{
	  /// No changes in points before rejected region.
	  while (this->points[i].pointIndex < uncertainStartPoint + 1)
	    {
	      ++i;
	      ++j;
	    }

	  /// Delete points in rejected region.
	  while (this->points[i].pointIndex <=
		 uncertainStartPoint + this->param.rejectSize)
	    ++i;
	  if (i >= this->points.size()) break;

	  /// Points for after rejected region.
	  this->points[j] = this->points[i];
	  this->points[j].pointIndex -= this->param.rejectSize;
	  this->points[j].numberOfElements = 1;

	  if (uncertainStartPoint == 0)
	    {
	      /// The case where the oldest datas were rejected.
	      surgery = false;
	      continue;
	    }

	  int surgeryStartPoint = this->points[j].pointIndex - this->param.rejectSize;
	  if (surgeryStartPoint < 0)
	    surgeryStartPoint = 0;
	  for (unsigned int k = 0; k < j ; ++k)
	    if (this->points[k].pointIndex > surgeryStartPoint)
	      this->points[j].numberOfElements++;

	  if (surgery)
	    {
	      /// The first surgery point after rejected region.
	      if (this->deepMemory[uncertainStartPoint - 1] !=
		  this->deepMemory[uncertainStartPoint])
		{
		  /// A new surgery point created by rejecting region.
		  surgeryPoint p0, p1;
		  p0.pointIndex = uncertainStartPoint;
		  p0.numberOfElements = this->points[j].numberOfElements;
		  p1 = this->points[j];
		  p1.numberOfElements += 1;
		  this->points[j] = p0;
		  /// Note that at least one surgery point was in
		  /// rejected region, therefore j <= i is maintained.
		  ++j;
		  this->points[j] = p1;
		}
	      surgery = false;
	    }
	}
      this->points.resize(j);


      /// Modify groups.
      j = 0;
      for (unsigned int i = 0; i < this->groups.size(); ++i)
	{
	  /// Delete any groups with no elements, and modify last elements.
	  if (this->groups[i].numberOfElements <= 0)
	    {
	      this->groupVectorMap.erase(this->groups[i].groupId);
	      continue;
	    }

	  this->groups[j] = this->groups[i];
	  this->groupVectorMap[this->groups[j].groupId] = j;
	  this->groups[j].elementsInObservation = 0;
	  ++j;
	}
      this->groups.resize(j);

      /// Find the dominant group.
      int dominantElements = 0;
      for (unsigned int i = 0; i < this->deepMemory.size(); ++i)
	{
	  /// subtraction returns an overflow
	  while (i + this->param.observations < this->deepMemory.size())
	    ++i;
	  int idInVector = groupVectorMap[this->deepMemory[i]];
	  int elements = this->groups[idInVector].elementsInObservation++;
	  if (elements >= dominantElements)
	    {
	      dominantElements = elements;
	      dominantGroup = this->groups[idInVector].groupId;
	    }
	}

      /// Swap dominant group to head.
      dataGroup dg = this->groups[0];
      this->groups[0] = this->groups[this->groupVectorMap[this->dominantGroup]];
      this->groupVectorMap[dg.groupId] = this->groupVectorMap[this->dominantGroup];
      this->groups[this->groupVectorMap[this->dominantGroup]] = dg;
      this->groupVectorMap[this->dominantGroup] = 0;
    }

    //////////////////////////////////////////////////
    template <typename T>
    void UncertainMemoryFilter<T>::PrintData()
    {
      int j = 0;
      for (unsigned int i = 0; i < this->deepMemory.size(); ++i)
	{
	  if (j < this->points.size())
	    if (this->points[j].pointIndex == i)
	      {
		std::cout << "\033[31m" << this->deepMemory[i] << "\033[0m ";
		++j;
		continue;
	      }
	  std::cout << this->deepMemory[i] << " ";
	}
      std::cout << "\n";
    }

    //////////////////////////////////////////////////
    template <typename T>
    int UncertainMemoryFilter<T>::GetDominantGroup()
    {
      if (this->groups.size() == 0)
	return -1;
      return this->dominantGroup;
    }

    //////////////////////////////////////////////////
    template <typename T>
    T UncertainMemoryFilter<T>::GetData()
    {
      return groups[groupVectorMap[this->dominantGroup]].lastElementData;
    }
  }
}

#endif
