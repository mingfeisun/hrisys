#ifndef _HRI_GREEDY_DECISION_FILTER_INL_H_
#define _HRI_GREEDY_DECISION_FILTER_INL_H_

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

#include "GreedyDecisionFilter.hh"

namespace gazebo
{
  namespace physics
  {
    //////////////////////////////////////////////////
    template <typename T>
    GreedyDecisionFilter<T>::GreedyDecisionFilter(sdf::ElementPtr _sdf) :
      param {_sdf->Get<int>("observations"),
	_sdf->Get<int>("greedyThre"), _sdf->Get<float>("clusterDist")}
    {
      this->state = {0};
    }

    //////////////////////////////////////////////////
    template <typename T>
    GreedyDecisionFilter<T>::~GreedyDecisionFilter()
    {
    }

    //////////////////////////////////////////////////
    template <typename T>
    void GreedyDecisionFilter<T>::AddData(T _newData)
    {
      /// The very start when no samples applied.
      if (this->sampleData.size() == 0)
	{
	  this->sampleData.push_back({0, _newData});
	  this->filteredData = _newData;
	  return;
	}
      /// Else, there is already a t=0 sample.

      /// Identify new data.
      if (this->state.possiblityOfNewDecision == 0)
	{
	  if (this->cost(_newData, this->filteredData) >
	      this->param.clusterDistance)
	    {
	      /// Data is out of variance.
	      this->sampleData.push_back({1, _newData});
	      this->state.possiblityOfNewDecision++;
	    }
	  /// Else, data is within variance.
	  return;
	}

      /// Check if next data is closer to out of variance or not.
      if (this->cost(_newData, this->sampleData[1].data) <
	  this->cost(_newData, this->sampleData[0].data))
	{
	  this->sampleData.push_back({1, _newData});
	  this->state.possiblityOfNewDecision++;
	}
      else
	{
	  this->sampleData.push_back({0, _newData});
	}

      /// If decision is true from greedy decision.
      if (this->state.possiblityOfNewDecision >=
	  this->param.greedyThreshold)
	{
	  this->state.possiblityOfNewDecision = 0;
	  /// Update filtered data.
	  this->filteredData = this->sampleData[1].data;
	  /// Create next samples.
	  this->sampleData[0] = {0, this->filteredData};
	  int j = 1;
	  for (unsigned int i = 2; i < this->sampleData.size(); ++i)
	    if (sampleData[i].group == 1)
	      {
		if (this->cost(this->sampleData[i].data, this->filteredData) >
		    this->param.clusterDistance)
		  {
		    T nextSample = this->sampleData[i].data;
		    if (this->state.possiblityOfNewDecision == 0)
		      {
			/// Data is out of variance.
			this->sampleData[j] = {1, nextSample};
			this->state.possiblityOfNewDecision++;
		      }
		    else
		      {
			if (this->cost(nextSample, this->sampleData[1].data) <
			    this->cost(nextSample, this->sampleData[0].data))
			  {
			    this->sampleData[j] = {1, nextSample};
			    this->state.possiblityOfNewDecision++;
			  }
			else
			  {
			    this->sampleData[j] = {0, nextSample};
			  }
		      }
		    ++j;
		  }
	      }
	  this->sampleData.resize(j);
	  return;
	}

      /// If decision is false from greedy decision.
      if (this->sampleData.size() >= this->param.observations)
	{
	  this->sampleData.resize(1);
	}

      /// Else, no decision can be made yet.
    }

    //////////////////////////////////////////////////
    template <typename T>
    void GreedyDecisionFilter<T>::SetCostFunction(std::function<double(T, T)> _fPtr)
    {
      this->cost = _fPtr;
    }

    //////////////////////////////////////////////////
    template <typename T>
    void GreedyDecisionFilter<T>::PrintData()
    {
      for (unsigned int i = 0; i < this->sampleData.size(); ++i)
	{
	  if (this->sampleData[i].group == 1)
	    {
	      std::cout << "\033[31m" << this->sampleData[i].group << "\033[0m ";
	      continue;
	    }
	  std::cout << this->sampleData[i].group << " ";
	}
      std::cout << "\n";
    }

    //////////////////////////////////////////////////
    template <typename T>
    T GreedyDecisionFilter<T>::GetData()
    {
      return this->filteredData;
    }
  }
}

#endif
