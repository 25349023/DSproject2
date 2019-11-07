#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "board.hpp"

class Robot {
    int battery;
    Board *board;

    void scanInitialize(){
        /// TODO: initialize the board's step and neighbor
    }
public:
    Robot(int bat, Board *brd = nullptr): battery(bat), board(brd) {
        scanInitialize();
    }

};


#endif