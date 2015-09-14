#ifndef _HRI_UNCERTAIN_MEMORY_FILTER_H_
#define _HRI_UNCERTAIN_MEMORY_FILTER_H_

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  namespace physics
  {
    template <typename T>
    class UncertainMemoryFilter
    {
    public: explicit UncertainMemoryFilter(sdf::ElementPtr _sdf);

    public: ~UncertainMemoryFilter();

    public: void AddData(T _newData);

    private: void UpdateMemory();

    public: int GetDominantGroup();

    public: T GetData();

    public: void PrintData();

    public: void SetCostFunction(std::function<double(T, T)> _fPtr);

    private:
      struct filterParams
      {
	const int observations;
	const int rejectSize;
	const int memorySize;
	const int uncertaintyThreshold;
	const double clusterDistance;
      };

    private:
      struct filterStateVariables
      {
	int previousClass;
	int nextGroupId;
      };

    private:
      struct dataGroup
      {
	int groupId;
	int numberOfElements;
	int elementsInObservation;
	T lastElementData;
      };
      
    private:
      struct surgeryPoint
      {
	int pointIndex;
	int numberOfElements;
      };

    private: int dominantGroup;

    private: filterStateVariables state;

    private: filterParams param;

    private: std::vector<int> deepMemory;

    private: std::vector<dataGroup> groups;

    private: std::map<int, int> groupVectorMap;

    private: std::vector<surgeryPoint> points;

    private: std::function<double(T, T)> cost;
    };

    template <typename T>
    using UncertainMemoryFilterPtr =  boost::shared_ptr<UncertainMemoryFilter<T> >;
  }
}

#include "UncertainMemoryFilter-inl.hh"

#endif
