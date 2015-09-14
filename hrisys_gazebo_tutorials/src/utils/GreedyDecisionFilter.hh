#ifndef _HRI_GREEDY_DECISION_FILTER_H_
#define _HRI_GREEDY_DECISION_FILTER_H_

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  namespace physics
  {
    template <typename T>
    class GreedyDecisionFilter
    {
    public: explicit GreedyDecisionFilter(sdf::ElementPtr _sdf);

    public: ~GreedyDecisionFilter();

    public: void AddData(T _newData);

    public: T GetData();

    public: void PrintData();

    public: void SetCostFunction(std::function<double(T, T)> _fPtr);

    private:
      struct filterParams
      {
	const int observations;
	const int greedyThreshold;
	const float clusterDistance;
      };

    private:
      struct filterStateVariables
      {
	int possiblityOfNewDecision;
      };

    private:
      struct dataElement
      {
	int group;
	T data;
      };

    private: filterStateVariables state;

    private: filterParams param;

    private: std::vector<dataElement> sampleData;

    private: T filteredData;

    private: std::function<double(T, T)> cost;
    };

    template <typename T>
    using GreedyDecisionFilterPtr = boost::shared_ptr<GreedyDecisionFilter<T> >;
  }
}

#include "GreedyDecisionFilter-inl.hh"

#endif
