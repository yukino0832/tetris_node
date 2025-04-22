#pragma once

#include "tetris.hpp"

class Planner {
public:
    Planner() = default;
    std::vector<commandType> commandSet;
    void plan(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet);

private:
    void planMandatory(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet);
    void planAdvanced(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet);
    void moveOnce(std::unordered_map<COLORS, std::vector<tetris>>& tetrisSet, COLORS type, int x, int y, int rotateTimes);
};