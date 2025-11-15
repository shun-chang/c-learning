#include "own_navigation.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace navigation{
bool own_navigation::configureMap(const std::vector<std::vector<int>> & map){
    if(map.empty()||map[0].empty()){
        return false;
    }
    _map.height=map.size();
    _map.width=map[0].size();
    _map.map=map;
    nav_path.clear();
    return true;
}

void own_navigation::displayMap() const{
    for(const auto row:_map.map){
        for(auto i=0;i<row.size();i++){
            std::cout<<row[i];
            if(i!=row.size()-1){
                std::cout<<" ";
            }
        }
        std::cout<<std::endl;
    }
}

bool own_navigation::isValidPose(const Pose2D & pose)const{
    if(pose.x<0||pose.x>=_map.width||pose.y<0||pose.y>=_map.height){
        return false;
    }
    int x=static_cast<int>(pose.x);
    int y=static_cast<int>(pose.y);
    return _map.map[y][x]!=0;
}

void own_navigation::victimMark(int x,int y){
    if(x>=0&&x<_map.width&&y>=0&&y<_map.height){
        _map.map[y][x]=3;
    }
}

std::vector<Pose2D> own_navigation::getNeighbors(const Pose2D &current) const{
    std::vector<Pose2D> neighbors;

    std::vector<std::pair<int,int>> dir={{0,-1},{0,1},{-1,0},{1,0}};
    std::vector<Heading> headings = {Heading::UP, Heading::DOWN, Heading::LEFT, Heading::RIGHT};

    for(auto i=0;i<4;i++){
        Pose2D neighbor={
            current.x+dir[i].first,
            current.y+dir[i].second,
            headings[i]
        };
        if(isValidPose(neighbor)){
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

navigation::Path own_navigation::reconstructPath(const std::map<Pose2D,Pose2D,Pose2DComp> &cameFrom,const Pose2D &goal)
{
    Path path;
    Pose2D current=goal;
    path.push_back(current);

    auto it = cameFrom.find(current);
    while(it!=cameFrom.end()){
        current=it->second;
        path.push_back(current);
        it = cameFrom.find(current);
    }
    std::reverse(path.begin(),path.end());
    return path;
}

bool own_navigation::planPathBFS(const Pose2D& start, const Pose2D& goal){
    nav_path.clear();
    if(!isValidPose(start) || !isValidPose(goal)){
        return false;
    }
    if(start.x==goal.x&&start.y==goal.y){
        nav_path.push_back(start);
        return true;
    }

    std::queue<Pose2D> queue;
    std::set<std::pair<double, double>> visited;
    std::map<Pose2D, Pose2D,Pose2DComp> cameFrom;

    queue.push(start);
    visited.insert({start.x,start.y});

    while(!queue.empty()){
        Pose2D current=queue.front();
        queue.pop();

        for(const auto neighbor : getNeighbors(current)){
            std::pair<double,double> pos={neighbor.x,neighbor.y};
            if(visited.find(pos)==visited.end()){
                cameFrom[neighbor]=current;
                visited.insert(pos);
                if (neighbor.x == goal.x && neighbor.y == goal.y) {
                    nav_path = reconstructPath(cameFrom, goal);
                    return true;
                }
                queue.push(neighbor);
            }
        }
    }
    return false;
}

bool own_navigation::move2Victim()const{
    if(nav_path.empty()){
        return false;
    }
    std::cout << "执行导航命令 ";
    for (size_t i = 1; i < nav_path.size(); ++i) {  
        const auto& pose = nav_path[i];
        std::cout << "当前位置 (" << pose.x << ", " << pose.y << ") ";
    }
    std::cout << "导航执行完毕！" << std::endl;
    return true;
}
}