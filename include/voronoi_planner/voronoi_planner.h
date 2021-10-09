#ifndef _VORONOI_PLANNER_H_
#define _VORONOI_PLANNER_H_

#include <cstdio>
#include <vector>
#include <cmath>
#include <string.h>
#include "voronoi_planner/heap.h"

namespace voronoi_planner
{
struct VoronoiData
{
  bool isVoronoi;
  double dist;
};

class Grid2DSearchState : public AbstractSearchState
{
public:
  /**
   * \brief coordinates
   */
  int x, y;

  /**
   * \brief search relevant data
   */
  int g;

  Grid2DSearchState* predecessor;

public:
  Grid2DSearchState()
  {
  }
  virtual ~Grid2DSearchState()
  {
  }
};

class VoronoiPlanner
{
public:
  VoronoiPlanner();
  ~VoronoiPlanner();

  bool search(int startX, int startY, int goalX, int goalY, int& pathCost, std::vector<std::pair<int, int> >* path,
              int size_x, int size_y, VoronoiData** voronoiDiagram, double circumscribed_radius);

private:
  bool SearchInVoronoi(int startX, int startY, int goalX, int goalY, int& pathCost,
                       std::vector<std::pair<int, int> >* path, VoronoiData** voronoiDiagram,
                       double circumscribed_radius);
  bool SearchShortestPathToVoronoi(int startX, int startY, int goalX, int goalY, int& voronoiGoalX, int& voronoiGoalY,
                                   int& pathCost, std::vector<std::pair<int, int> >* path, VoronoiData** voronoiDiagram,
                                   double circumscribed_radius);

private:
  bool WithinSearchSpace(int x, int y) const
  {
    if (x < 0 || x >= m_Width || y < 0 || y >= m_Height)
      return false;

    return true;
  }

  int index(int x, int y) const
  {
    return x + y * m_Width;
  }

  int heuristic(int x, int y, int goalX, int goalY) const
  {
    return 10 * std::max(abs(x - goalX), abs(y - goalY));
  }

  void CreateSearchSpace();
  void ReInitializeSearchSpace();
  void InitializeSearchState(Grid2DSearchState* searchState);
  void computedxy();
  void destroy();

private:
  int m_Width, m_Height;
  Grid2DSearchState** m_SearchSpace;
  CIntHeap* m_OPEN;

  // largest optimal g-value computed by search
  int largestcomputedoptf_;

  int dx_[8];
  int dy_[8];
  int dxy_cost_[8];
  bool use_heuristic_;
};

}  // namespace voronoi_planner

#endif  // _VORONOI_PLANNER_H_