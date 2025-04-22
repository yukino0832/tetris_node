#include "../include/planner.hpp"

/**
 * @brief 根据检测到的方块规划如何摆放
 * @param[in] tetrisSet 检测到的方块集合
 */
void Planner::plan(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet) {
    if (Param::phase == PHASES::FIRST) {
        planMandatory(tetrisSet);
    } else {
        planAdvanced(tetrisSet);
    }
}

/**
 * @brief 必做任务规划(纯手动放，后续可以用进阶任务规划代替)
 * @param[in] tetrisSet 检测到的方块集合
 */
void Planner::planMandatory(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet) {
    moveOnce(tetrisSet, COLORS::PURPLE, 10, 14, 3);
    moveOnce(tetrisSet, COLORS::RED, 6, 14, 0);
    moveOnce(tetrisSet, COLORS::YELLOW, 4, 14, 0);
    moveOnce(tetrisSet, COLORS::BROWN, 1, 13, 1);
    moveOnce(tetrisSet, COLORS::GREEN, 2, 12, 3);
    moveOnce(tetrisSet, COLORS::YELLOW, 7, 13, 0);
    moveOnce(tetrisSet, COLORS::ORANGE, 8, 12, 0);
    moveOnce(tetrisSet, COLORS::PURPLE, 7, 12, 0);
    moveOnce(tetrisSet, COLORS::BLUE, 1, 10, 3);
    moveOnce(tetrisSet, COLORS::BROWN, 3, 10, 1);
    moveOnce(tetrisSet, COLORS::YELLOW, 7, 11, 0);
    moveOnce(tetrisSet, COLORS::BROWN, 9, 11, 0);
    moveOnce(tetrisSet, COLORS::BLUE, 5, 9, 0);
    moveOnce(tetrisSet, COLORS::PURPLE, 8, 9, 2);
    moveOnce(tetrisSet, COLORS::GREEN, 9, 9, 3);
    moveOnce(tetrisSet, COLORS::RED, 1, 7, 1);
    moveOnce(tetrisSet, COLORS::RED, 3, 8, 0);
    moveOnce(tetrisSet, COLORS::YELLOW, 8, 8, 0);
    moveOnce(tetrisSet, COLORS::ORANGE, 2, 6, 0);
    moveOnce(tetrisSet, COLORS::PURPLE, 4, 5, 1);
    moveOnce(tetrisSet, COLORS::ORANGE, 5, 6, 0);
    moveOnce(tetrisSet, COLORS::BLUE, 8, 6, 1);
    moveOnce(tetrisSet, COLORS::ORANGE, 2, 6, 0);
    moveOnce(tetrisSet, COLORS::YELLOW, 3, 4, 2);
    moveOnce(tetrisSet, COLORS::GREEN, 7, 4, 0);
    moveOnce(tetrisSet, COLORS::BLUE, 10, 5, 1);
    moveOnce(tetrisSet, COLORS::RED, 2, 3, 0);
    moveOnce(tetrisSet, COLORS::GREEN, 5, 3, 3);
    moveOnce(tetrisSet, COLORS::ORANGE, 7, 2, 0);
    moveOnce(tetrisSet, COLORS::BLUE, 10, 3, 1);
    moveOnce(tetrisSet, COLORS::RED, 2, 2, 0);
    moveOnce(tetrisSet, COLORS::PURPLE, 6, 1, 2);
    moveOnce(tetrisSet, COLORS::BROWN, 9, 1, 2);
}

/**
 * @brief 进阶任务规划
 * @param[in] tetrisSet 检测到的方块集合
 */
void Planner::planAdvanced(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet) {
    //TODO: 进阶任务规划
    // moveOnce函数用于将某种方块移到(x, y)并旋转rotateTimes次
    // moveOnce(tetrisSet, COLORS::PURPLE, x, y, rotateTimes);
}

/**
 * @brief 移动一个方块到目标位置
 * @param[in] tetrisSet 方块集合
 * @param[in] type 方块类型
 * @param[in] x,y 目标位置坐标
 * @param[in] rotateTimes 顺时针旋转到目标状态所需次数(0-3)
 */
void Planner::moveOnce(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet, COLORS type, int x, int y, int rotateTimes) {
    commandType command(x, y);
    command.Tetris = std::move(tetrisSet[type].back());
    tetrisSet[type].pop_back();
    command.angle = command.Tetris.angleToHorizontal + rotateTimes * M_PI / 2;
    if (command.angle > M_PI) {
        command.angle -= 2 * M_PI;
    }
    commandSet.push_back(std::move(command));
}