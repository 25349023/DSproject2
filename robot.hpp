#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "board.hpp"

struct Point
{
    short x, y;
    Point(int _x = 0, int _y = 0): x(_x), y(_y) {}

    bool operator ==(const Point &oth){
        return x == oth.x && y == oth.y;
    }
    bool operator !=(const Point &oth){
        return x != oth.x || y != oth.y;
    }
};

class Robot {
    int battery;
    Board *board;
    Point position;
    int cells_to_clean;

    void scan_initialize();
    bool can_visit(const Cell &c);
    Point pick_by_distance(const Point &p);
    Point pick_by_neighbor(const Point &p);

public:
    Robot(int bat, Point charger, Board *brd = nullptr):
        battery(bat), board(brd), position(charger), cells_to_clean(0) {
        scan_initialize();
    }

    Point pick_one_cell();

    bool is_finished(){
        return cells_to_clean == 0;
    }
};

bool Robot::can_visit(const Cell &c){
    return c.kind == '0' && c.steps == 0;
}


void Robot::scan_initialize(){
    Point *que = new Point[board->rows * board->cols];
    int front = 0, tail = 0;
    que[front++] = position;
    short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    using std::cout;
    using std::endl;

    // cout << "start BFS" << endl;
    while (front > tail){
        Point curr = que[tail++];
        // cout << "visit point " << curr.x << " " << curr.y << endl;
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (next.x < 0 || next.x >= board->rows ||
                next.y < 0 || next.y >= board->cols){
                continue;
            }
            // cout << "try point " << next.x << " " << next.y << endl;
            if (can_visit(board->floor[next.x][next.y])){
                // cout << "add point " << next.x << " " << next.y << endl;
                que[front++] = next;
                board->floor[next.x][next.y].steps = 
                    board->floor[curr.x][curr.y].steps + 1;
                for (int i = 0; i < 4; i++){
                    int tx = next.x + dir[i][0], ty = next.y + dir[i][1];
                    if (board->floor[tx][ty].kind == '0'){
                        board->floor[next.x][next.y].neighbor++; 
                    }
                }
            }
        }
    }
    delete [] que;
}

Point Robot::pick_one_cell(){

}

#endif