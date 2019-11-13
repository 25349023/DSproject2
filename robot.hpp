#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "board.hpp"

class Robot {
    int battery, full_bat;
    Board *board;
    Point position, last_dirty_cell;
    const Point charger;
    Point *que;
    int cells_to_clean;
    bool low_battery, finding_path;
    
    void scan_initialize();

    bool can_visit(const Cell &c);
    bool out_of_bound(const Point& p);
    bool out_of_bound(const short x, const short y);

    Point pick_one_cell();
    Point pick_by_distance();
    Point get_closed_to_ldc();
    Point find_near();

    bool check_for_battery();
    void decrease_neighbor();
    void record_last_dirty();
    void find_dirty_cell();
    void plan_path_to_dirty();
    void reset_path();

public:
    Robot(int bat, Point chgr, Board *brd = nullptr):
        battery(bat), full_bat(bat), board(brd), position(chgr), last_dirty_cell(),
        charger(chgr), que(new Point[board->rows * board->cols + 10]), 
        cells_to_clean(0), low_battery(false), finding_path(false) {
        (*board)[chgr].cleaned = true;
        scan_initialize();
    }

    ~Robot(){
        delete [] que;
    }

    Point sweep_one_cell();

    bool is_finished(){
        return cells_to_clean == 0 && position == charger;
    }
};

bool Robot::can_visit(const Cell &c){
    return c.kind == '0' && c.steps == 0;
}

bool Robot::out_of_bound(const Point& p){
    if (p.x < 0 || p.x >= board->rows ||
        p.y < 0 || p.y >= board->cols){
        return true;
    }
    return false;
}

bool Robot::out_of_bound(const short x, const short y){
    if (x < 0 || x >= board->rows ||
        y < 0 || y >= board->cols){
        return true;
    }
    return false;
}


void Robot::scan_initialize(){
    int front = 0, tail = 0;
    que[front++] = position;
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    while (front > tail){
        Point curr = que[tail++];
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next)){
                continue;
            }
            if (can_visit((*board)[next])){
                que[front++] = next;
                (*board)[next].steps = (*board)[curr].steps + 1;
                for (int i = 0; i < 4; i++){
                    int tx = next.x + dir[i][0], ty = next.y + dir[i][1];
                    if ((*board)[tx][ty].kind == '0'){
                        (*board)[next].neighbor++; 
                    }
                }
                cells_to_clean++;
            }
        }
    }
}

bool Robot::check_for_battery(){
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
        if (out_of_bound(nx, ny)){
            continue;
        }
        if ((*board)[nx][ny].steps >= battery){
            low_battery = true;
            return true;
        }
    }
    return false;
}


Point Robot::pick_by_distance(){
    Point target(position);
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    int max_step = 0, min_neighbor = 5;

    for (int k = 0; k < 4; k++){
        Point next(position.x + dir[k][0], position.y + dir[k][1]);
        if (out_of_bound(next) || !(*board)[next].need_to_clean()){
            continue;
        }

        if ( ((*board)[next].steps > max_step) || 
             ((*board)[next].steps == max_step &&
                (*board)[next].neighbor < min_neighbor) ){
            max_step = (*board)[next].steps;
            min_neighbor = (*board)[next].neighbor;
            target = next;
        }
    }

    return target;
}

Point Robot::find_near(){
    Point target(position);
    int min_step = (*board)[position].steps;
    int min_neighbor = 5;
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        Point next(position.x + dir[k][0], position.y + dir[k][1]);
        if (out_of_bound(next) || !(*board)[next].can_walk()){
            continue;
        }
        if ((*board)[next].steps < min_step){
            target = next;
            min_step = (*board)[next].steps;
            if ((*board)[next].cleaned){
                min_neighbor = 5;
            }
            else {
                min_neighbor = (*board)[next].neighbor;
            }
        }
        else if ((*board)[next].steps == min_step){
            if (!(*board)[next].cleaned && 
                (*board)[next].neighbor < min_neighbor){
                target = next;
                min_step = (*board)[next].steps;
                min_neighbor = (*board)[next].neighbor;
            }
        }
    }
    return target;
}


Point Robot::get_closed_to_ldc(){
    int min_step = (*board)[position].step_wrt_ldc;
    Point target(position);
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        Point next(position.x + dir[k][0], position.y + dir[k][1]);
        if (out_of_bound(next) || !(*board)[next].can_walk() ||
            (*board)[next].step_wrt_ldc < 0){
            continue;
        }

        if ((*board)[next].step_wrt_ldc < min_step){
            min_step = (*board)[next].step_wrt_ldc;
            target = next;
        }
    }
    return target;
}


Point Robot::pick_one_cell(){
    Point target;
    if (low_battery || check_for_battery() || cells_to_clean == 0){
        target = find_near();
    }
    else if (finding_path){
        target = get_closed_to_ldc();
    }
    else {
        target = pick_by_distance();
        if (target == position){
            if ((*board)[last_dirty_cell].neighbor <= 0){
                find_dirty_cell();
            }
            plan_path_to_dirty();
            finding_path = true;
            target = get_closed_to_ldc();
        }
        if (target == position){
            target = find_near();
        }
    }

    return target;
}

void Robot::decrease_neighbor(){
    if (position == charger){
        return;
    }
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
        if (!out_of_bound(nx, ny) && (*board)[nx][ny].kind == '0'){
            (*board)[nx][ny].neighbor--;
        }
    }
}

void Robot::record_last_dirty(){
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    if ((*board)[position].neighbor > 1){
        last_dirty_cell = position;
    }
    else if (low_battery && (*board)[position].neighbor == 1){
        for (int k = 0; k < 4; k++){
            short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
            if (out_of_bound(nx, ny) || !(*board)[nx][ny].need_to_clean()){
                continue;
            }
            if ((*board)[nx][ny].steps > (*board)[position].steps){
                last_dirty_cell = position;
            }
        }
    }
}

void Robot::find_dirty_cell(){
    bool **visited = new bool*[board->rows];
    for (int i = 0; i < board->rows; i++){
        visited[i] = new bool[board->cols]{};
    }
    int front = 0, tail = 0;
    que[front++] = position;
    visited[position.x][position.y] = true;
    const short dir[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};

    bool flag = false;
    while (front > tail && !flag){
        Point curr = que[tail++];
        for (int k = 0; k < 4 && !flag; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next) || !(*board)[next].can_walk()){
                continue;
            }
            if (!visited[next.x][next.y]){
                que[front++] = next;
                visited[next.x][next.y] = true;
                if (!(*board)[next].cleaned){
                    last_dirty_cell = next;
                    flag = true;
                }
            }
        }
    }
    for (int i = 0; i < board->rows; i++){
        delete [] visited[i];
    }
    delete [] visited;
}

void Robot::plan_path_to_dirty(){
    int front = 0, tail = 0;
    const short dir[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
    que[front++] = last_dirty_cell;
    (*board)[last_dirty_cell].step_wrt_ldc = 0;

    while (front > tail){
        Point curr = que[tail++];
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next) || !(*board)[next].can_walk()){
                continue;
            }
            if ((*board)[next].step_wrt_ldc == -1){  // not visited
                que[front++] = next;
                (*board)[next].step_wrt_ldc = (*board)[curr].step_wrt_ldc + 1;
                if (next == position){
                    return;
                }
            }
        }
    }
}

void Robot::reset_path(){
    int front = 0, tail = 0;
    const short dir[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
    que[front++] = last_dirty_cell;
    (*board)[last_dirty_cell].step_wrt_ldc = -1;

    while (front > tail){
        Point curr = que[tail++];
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next) || !(*board)[next].can_walk()){
                continue;
            }
            if ((*board)[next].step_wrt_ldc != -1){  // not visited
                que[front++] = next;
                (*board)[next].step_wrt_ldc = -1;
            }
        }
    }
}

Point Robot::sweep_one_cell(){
    position = pick_one_cell();
    if (position == last_dirty_cell){
        reset_path();
        finding_path = false;
    }
    if (!(*board)[position].cleaned){
        (*board)[position].cleaned = true;
        cells_to_clean--;
        decrease_neighbor();
    }
    if (!finding_path){
        record_last_dirty();
    }

    battery--;
    if (position == charger){
        battery = full_bat;
        low_battery = false;
        if (finding_path){
            reset_path();
            plan_path_to_dirty();
        }
    }
    if (battery <= 0){
        throw "out of power";
    }

    return position;
}

#endif