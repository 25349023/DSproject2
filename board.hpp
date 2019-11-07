#ifndef BOARD_HPP
#define BOARD_HPP

struct Cell {
    char kind;
    int steps;
    int neighbor;
    bool cleaned;

    Cell(): kind('1'), steps(0), neighbor(0), cleaned(false) {}

    bool isCharger() {
        return kind == 'R';
    }
};

struct Point
{
    int x, y;
    Point(int _x = 0, int _y = 0): x(_x), y(_y) {}
};

class Robot;

class Board {
    int rows, cols;

public:
    Cell **floor; 

    Board(int r, int c): rows(r), cols(c), floor(nullptr) {
        floor = new Cell* [r];
        for (int i = 0; i < r; i++){
            floor[i] = new Cell[c];
        }
    }

    ~Board(){
        for (int i = 0; i < rows; i++){
            delete[] floor[i];
        }
        delete[] floor;
    }

    friend Robot;
};

#endif