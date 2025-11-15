#ifndef NAVIGATION_PLANNER_HPP
#define NAVIGATION_PLANNER_HPP
#include <vector>
#include <iostream>
#include <string>

namespace navigation {
enum class Heading { STOP, UP, DOWN, LEFT, RIGHT };
struct Pose2D {
    double x{};
    double y{};
    Heading heading = Heading::STOP;  // 暂停或上下左右方向
    Pose2D() = default;
    Pose2D(double x_, double y_, Heading h = Heading::STOP)
        : x(x_), y(y_), heading(h) {}
};
struct GridMap {
    int width{};
    int height{};
    double resolution_m{1.0};
    std::vector<std::vector<int>> map;
    int size() const { return width * height; }
};
struct NavigationGoal {
    Pose2D target_pose{};
    GridMap map{};
};

using Path = std::vector<Pose2D>;

class NavigationPlanner {
public:
    NavigationPlanner() = default;
    virtual ~NavigationPlanner() = default;
    virtual bool configureMap(const std::vector<std::vector<int>>& map) = 0;
    virtual void displayMap() const = 0;
    virtual bool planPathBFS(const Pose2D& start, const Pose2D& goal) = 0;
    virtual bool isValidPose(const Pose2D& pose) const = 0;
    virtual bool move2Victim() const = 0;
    const navigation::Path& getNavPath() const { return nav_path; }
    const navigation::GridMap& getMap() const { return _map; }

protected:
    GridMap _map;
    Path nav_path;
};
}  // namespace navigation

#endif
