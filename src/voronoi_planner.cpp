#include <algorithm>
#include <chrono>
#include "voronoi_planner/voronoi_planner.h"

namespace voronoi_planner
{
VoronoiPlanner::VoronoiPlanner() : m_SearchSpace(NULL), m_OPEN(NULL), use_heuristic_(true), m_Width(0), m_Height(0)
{
  largestcomputedoptf_ = 0;

  // compute dx, dy, dxintersects and dyintersects arrays
  computedxy();
}

VoronoiPlanner::~VoronoiPlanner()
{
  destroy();
}

void VoronoiPlanner::destroy()
{
  if (m_OPEN)
  {
    m_OPEN->makeemptyheap();
    delete m_OPEN;
    m_OPEN = NULL;
  }

  if (m_SearchSpace)
  {
    for (int x = 0; x < m_Width; x++)
      delete[] m_SearchSpace[x];

    delete[] m_SearchSpace;
    m_SearchSpace = NULL;
  }
}

void VoronoiPlanner::InitializeSearchState(Grid2DSearchState* searchState)
{
  searchState->g = INFINITECOST;
  searchState->heapindex = 0;
  searchState->predecessor = NULL;
}

void VoronoiPlanner::CreateSearchSpace()
{
  m_SearchSpace = new Grid2DSearchState*[m_Width];
  for (int x = 0; x < m_Width; x++)
  {
    m_SearchSpace[x] = new Grid2DSearchState[m_Height];
    for (int y = 0; y < m_Height; y++)
    {
      m_SearchSpace[x][y].x = x;
      m_SearchSpace[x][y].y = y;
      InitializeSearchState(&m_SearchSpace[x][y]);
    }
  }
}

void VoronoiPlanner::ReInitializeSearchSpace()
{
  for (int x = 0; x < m_Width; x++)
  {
    for (int y = 0; y < m_Height; y++)
    {
      InitializeSearchState(&m_SearchSpace[x][y]);
    }
  }
}

void VoronoiPlanner::computedxy()
{
  // initialize some constants for 2D search
  dx_[0] = 1;
  dy_[0] = 1;
  dx_[1] = 1;
  dy_[1] = 0;
  dx_[2] = 1;
  dy_[2] = -1;
  dx_[3] = 0;
  dy_[3] = 1;
  dx_[4] = 0;
  dy_[4] = -1;
  dx_[5] = -1;
  dy_[5] = 1;
  dx_[6] = -1;
  dy_[6] = 0;
  dx_[7] = -1;
  dy_[7] = -1;

  // compute distances
  for (int dind = 0; dind < 8; dind++)
  {
    if (dx_[dind] != 0 && dy_[dind] != 0)
    {
      dxy_cost_[dind] = 14;
    }
    else
      dxy_cost_[dind] = 10;
  }
}

bool VoronoiPlanner::search(int startX, int startY, int goalX, int goalY, int& pathCost,
                            std::vector<std::pair<int, int> >* path, int size_x, int size_y,
                            VoronoiData** voronoiDiagram, double circumscribed_radius)
{
  if (size_x != m_Width || size_y != m_Height)
  {
    destroy();

    m_Width = size_x;
    m_Height = size_y;

    m_OPEN = new CIntHeap(m_Width * m_Height);
    CreateSearchSpace();
  }

  std::vector<std::pair<int, int> > path1, path2, path3;
  int pathCost1, pathCost2, pathCost3;
  int voronoiStartX, voronoiStartY;
  if (!SearchShortestPathToVoronoi(startX, startY, goalX, goalY, voronoiStartX, voronoiStartY, pathCost1, &path1,
                                   voronoiDiagram, circumscribed_radius))
  {
    printf("Failed to fine the path from start to voronoi\n");
    return false;
  }
  std::reverse(path1.begin(), path1.end());

  if (voronoiStartX == goalX && voronoiStartY == goalY)
  {
    path->clear();
    for (int i = 0; i < (int)path1.size(); i++)
      path->push_back(path1[i]);

    pathCost = pathCost1;

    return true;
  }

  int voronoiGoalX, voronoiGoalY;
  if (!SearchShortestPathToVoronoi(goalX, goalY, startX, startY, voronoiGoalX, voronoiGoalY, pathCost3, &path3,
                                   voronoiDiagram, circumscribed_radius))
  {
    printf("Failed to the find the path from voronoi to goal\n");
    return false;
  }

  if (voronoiGoalX == startX && voronoiGoalY == startY)
  {
    path->clear();
    for (int i = 0; i < (int)path3.size(); i++)
      path->push_back(path3[i]);

    pathCost = pathCost3;

    return true;
  }

  if (!SearchInVoronoi(voronoiStartX, voronoiStartY, voronoiGoalX, voronoiGoalY, pathCost2, &path2, voronoiDiagram,
                       circumscribed_radius))
  {
    printf("Failed to find the path from voronoi start to voronoi goal\n");
    return false;
  }

  path1.insert(path1.end(), path2.begin(), path2.end());
  path1.insert(path1.end(), path3.begin(), path3.end());

  path->clear();
  for (int i = 0; i < (int)path1.size(); i++)
    path->push_back(path1[i]);

  pathCost = pathCost1 + pathCost2 + pathCost3;

  return true;
}

bool VoronoiPlanner::SearchInVoronoi(int startX, int startY, int goalX, int goalY, int& pathCost,
                                     std::vector<std::pair<int, int> >* path, VoronoiData** voronoiDiagram,
                                     double circumscribed_radius)
{
  const auto start_t = std::chrono::system_clock::now();

  ReInitializeSearchSpace();
  m_OPEN->makeemptyheap();

  Grid2DSearchState* searchExpState = NULL;
  Grid2DSearchState* searchSuccState = NULL;
  Grid2DSearchState* searchGoalState = &m_SearchSpace[goalX][goalY];
  int key;

  searchExpState = &m_SearchSpace[startX][startY];
  searchExpState->g = 0;
  key = searchExpState->g;
  if (use_heuristic_)
    key = key + heuristic(startX, startY, goalX, goalY);

  m_OPEN->insertheap(searchExpState, key);

  int numOfExpands = 0;
  char* pbClosed = (char*)calloc(1, m_Width * m_Height);

  while (!m_OPEN->emptyheap() && searchGoalState->g > m_OPEN->getminkeyheap())
  {
    searchExpState = dynamic_cast<Grid2DSearchState*>(m_OPEN->deleteminheap());

    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;
    numOfExpands++;
    // close the state
    pbClosed[exp_x + m_Width * exp_y] = 1;

    for (int dir = 0; dir < 8; dir++)
    {
      int new_x = exp_x + dx_[dir];
      int new_y = exp_y + dy_[dir];

      if (!WithinSearchSpace(new_x, new_y))
        continue;

      if (pbClosed[new_x + m_Width * new_y] == 1)
        continue;

      if (!voronoiDiagram[new_x][new_y].isVoronoi)
        continue;

      double dist = voronoiDiagram[new_x][new_y].dist;
      if (dist < circumscribed_radius)
        continue;

      int cost = dxy_cost_[dir];

      searchSuccState = &m_SearchSpace[new_x][new_y];
      if (searchSuccState->g > searchExpState->g + cost)
      {
        searchSuccState->g = searchExpState->g + cost;
        searchSuccState->predecessor = searchExpState;

        key = searchSuccState->g;
        if (use_heuristic_)
          key = key + heuristic(new_x, new_y, goalX, goalY);

        if (searchSuccState->heapindex == 0)
          m_OPEN->insertheap(searchSuccState, key);
        else
          m_OPEN->updateheap(searchSuccState, key);
      }
    }  // over successors
  }    // while

  // set lower bounds for the remaining states
  if (!m_OPEN->emptyheap())
    largestcomputedoptf_ = m_OPEN->getminkeyheap();
  else
    largestcomputedoptf_ = INFINITECOST;

  free(pbClosed);
  const auto end_t = std::chrono::system_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;
  printf("Time used to search voronoi path=%fms\n", timediff.count() * 1000);

  // printf(
  //     "# of expands during 2dgridsearch=%d time=%f msecs 2Dsolcost_inmm=%d "
  //     "largestoptfval=%d (start=%d %d goal=%d %d)\n",
  //     numOfExpands, timediff.count() * 1000, m_SearchSpace[m_GoalX][m_GoalY].g, largestcomputedoptf_, m_StartX,
  //     m_StartY, m_GoalX, m_GoalY);

  path->clear();
  Grid2DSearchState* searchPredState = searchGoalState;
  path->push_back(std::make_pair(searchPredState->x, searchPredState->y));
  while (searchPredState->predecessor != NULL)
  {
    searchPredState = searchPredState->predecessor;
    path->push_back(std::make_pair(searchPredState->x, searchPredState->y));
  }
  std::reverse(path->begin(), path->end());

  return true;
}

bool VoronoiPlanner::SearchShortestPathToVoronoi(int startX, int startY, int goalX, int goalY, int& voronoiGoalX,
                                                 int& voronoiGoalY, int& pathCost,
                                                 std::vector<std::pair<int, int> >* path, VoronoiData** voronoiDiagram,
                                                 double circumscribed_radius)
{
  if (!WithinSearchSpace(startX, startY))
  {
    throw std::runtime_error("the start is out of range");
    return false;
  }

  const auto start_t = std::chrono::system_clock::now();

  ReInitializeSearchSpace();
  m_OPEN->makeemptyheap();

  Grid2DSearchState* searchExpState = NULL;
  Grid2DSearchState* searchSuccState = NULL;

  searchExpState = &m_SearchSpace[startX][startY];
  searchExpState->g = 0;
  m_OPEN->insertheap(searchExpState, searchExpState->g);

  int numOfExpands = 0;
  char* pbClosed = (char*)calloc(1, m_Width * m_Height);

  while (!m_OPEN->emptyheap())
  {
    searchExpState = dynamic_cast<Grid2DSearchState*>(m_OPEN->deleteminheap());

    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;

    if ((exp_x == goalX && exp_y == goalY) || (voronoiDiagram[exp_x][exp_y].isVoronoi))
    {
      voronoiGoalX = exp_x;
      voronoiGoalY = exp_y;
      break;
    }

    numOfExpands++;
    // close the state
    pbClosed[exp_x + m_Width * exp_y] = 1;

    for (int dir = 0; dir < 8; dir++)
    {
      int new_x = exp_x + dx_[dir];
      int new_y = exp_y + dy_[dir];

      if (!WithinSearchSpace(new_x, new_y))
        continue;

      if (pbClosed[new_x + m_Width * new_y] == 1)
        continue;

      double dist = voronoiDiagram[new_x][new_y].dist;
      if (dist < circumscribed_radius)
        continue;

      int cost = dxy_cost_[dir];

      searchSuccState = &m_SearchSpace[new_x][new_y];
      if (searchSuccState->g > searchExpState->g + cost)
      {
        searchSuccState->g = searchExpState->g + cost;
        searchSuccState->predecessor = searchExpState;

        if (searchSuccState->heapindex == 0)
          m_OPEN->insertheap(searchSuccState, searchSuccState->g);
        else
          m_OPEN->updateheap(searchSuccState, searchSuccState->g);
      }
    }  // over successors
  }    // while

  // set lower bounds for the remaining states
  if (!m_OPEN->emptyheap())
    largestcomputedoptf_ = m_OPEN->getminkeyheap();
  else
    largestcomputedoptf_ = INFINITECOST;

  free(pbClosed);
  const auto end_t = std::chrono::system_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;

  // printf(
  //     "# of expands during 2dgridsearch=%d time=%f msecs 2Dsolcost_inmm=%d "
  //     "largestoptfval=%d (start=%d %d goal=%d %d)\n",
  //     numOfExpands, timediff.count() * 1000, m_SearchSpace[m_GoalX][m_GoalY].g, largestcomputedoptf_, m_StartX,
  //     m_StartY, m_GoalX, m_GoalY);

  path->clear();
  Grid2DSearchState* searchGoalState = &m_SearchSpace[voronoiGoalX][voronoiGoalY];
  Grid2DSearchState* searchPredState = searchGoalState;
  path->push_back(std::make_pair(searchPredState->x, searchPredState->y));
  while (searchPredState->predecessor != NULL)
  {
    searchPredState = searchPredState->predecessor;
    path->push_back(std::make_pair(searchPredState->x, searchPredState->y));
  }

  return true;
}

}  // namespace voronoi_planner
