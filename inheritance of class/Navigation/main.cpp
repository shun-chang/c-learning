#include "own_navigation.hpp"
#include <iostream>
#include <windows.h>


int main(){
    SetConsoleOutputCP(936);
    SetConsoleCP(936);
    std::cout<<"测试代码输出"<<std::endl;
    std::vector<std::vector<int>> map_date={
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1},
        {1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1}, {1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };
    navigation::own_navigation planner;
    bool configsuccess = planner.configureMap(map_date);
    if(configsuccess){
        std::cout<<"地图配置成功： 宽="<<planner.getMap().width;
        std::cout<<"，高="<<planner.getMap().height<<std::endl;
        planner.displayMap();
        std::cout<<"环境地图生成成功！"<<std::endl;

    }
    else {
        std::cout << "地图配置失败！" << std::endl;
        return 1;
    }
    int victimX = 8, victimY = 8;
    planner.victimMark(victimX, victimY);
    std::cout << "被困者位置标记成功 (" << victimX << ", " << victimY << ")" << std::endl;
    planner.displayMap();
    std::cout << "被困者位置设置成功！" << std::endl;
    navigation::Pose2D start(0, 0);
    navigation::Pose2D goal(victimX, victimY);

    bool planSuccess = planner.planPathBFS(start, goal);
    if (planSuccess) {
        std::cout << "规划成功！" << std::endl;
        planner.move2Victim();
    } else {
        std::cout << "规划失败，无法找到路径！" << std::endl;
    }
    return 0;
}