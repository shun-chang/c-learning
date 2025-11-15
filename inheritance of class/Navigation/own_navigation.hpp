#ifndef OWN_NAVIGATION_HPP
#define OWN_NAVIGATION_HPP

#include "navigation_planner.hpp"
#include <queue>
#include <set>
#include <stack>
#include <vector>
#include <map>

namespace navigation {

struct Pose2DComp {
    bool operator()(const Pose2D& a, const Pose2D& b) const {
        if (a.x != b.x) { 
            return a.x < b.x;
        }
        return a.y < b.y; 
    }
};


class own_navigation : public NavigationPlanner {
public:
  own_navigation() = default;
  ~own_navigation() override = default;

  bool configureMap(const std::vector<std::vector<int>> &map) override;
  void displayMap() const override;
  bool planPathBFS(const Pose2D &start, const Pose2D &goal) override;
  bool isValidPose(const Pose2D &pose) const override;
  bool move2Victim() const override;

  void victimMark(int x, int y);

private:
  std::vector<Pose2D> getNeighbors(const Pose2D &current) const;

  Path reconstructPath(const std::map<Pose2D,Pose2D,Pose2DComp> &cameFrom,const Pose2D &goal);
};
} // namespace navigation
#endif
