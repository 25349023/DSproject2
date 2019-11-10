#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <cmath>
#include "board.hpp"
using std::cout;
using std::endl;
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
    Point operator -(const Point &oth){
        return Point(x - oth.x, y - oth.y);
    }

    int distance(const Point &oth){
        using std::abs;
        return abs(x - oth.x) + abs(y - oth.y);
    }
};

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
    int step_diff(const Point &rhs);

    Point pick_one_cell();
    void decrease_neighbor();
    void record_last_dirty();

    Point pick_by_distance();
    Point get_closed_to_ldc();
    Point find_near();

    void find_dirty_cell();
    void plan_path_to_dirty();
    void reset_path();
    bool check_for_battery();

public:
    Robot(int bat, Point chgr, Board *brd = nullptr):
        battery(bat), full_bat(bat), board(brd), position(chgr), last_dirty_cell(),
        charger(chgr), que(new Point[board->rows * board->cols]), 
        cells_to_clean(0), low_battery(false), finding_path(false) {
        board->floor[chgr.x][chgr.y].cleaned = true;
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

int Robot::step_diff(const Point &rhs){
    using std::abs;
    return abs(board->floor[last_dirty_cell.x][last_dirty_cell.y].step_wrt_ldc -
        board->floor[rhs.x][rhs.y].step_wrt_ldc);
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
            if (can_visit(board->floor[next.x][next.y])){
                que[front++] = next;
                board->floor[next.x][next.y].steps = 
                    board->floor[curr.x][curr.y].steps + 1;
                for (int i = 0; i < 4; i++){
                    int tx = next.x + dir[i][0], ty = next.y + dir[i][1];
                    if (board->floor[tx][ty].kind == '0'){
                        board->floor[next.x][next.y].neighbor++; 
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
        if (board->floor[nx][ny].steps >= battery){
            cout << "low battery" << endl;
            low_battery = true;
            return true;
        }
    }
    return false;
}


Point Robot::pick_by_distance(){
    cout << "pick by dis  ";
    Point target(position);
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    int max_step = 0, min_neighbor = 5;

    for (int k = 0; k < 4; k++){
        short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
        if (out_of_bound(nx, ny) || !board->floor[nx][ny].need_to_clean()){
            continue;
        }

        if ( (board->floor[nx][ny].steps > max_step) || 
             (board->floor[nx][ny].steps == max_step &&
                board->floor[nx][ny].neighbor < min_neighbor) ){
            max_step = board->floor[nx][ny].steps;
            min_neighbor = board->floor[nx][ny].neighbor;
            target.x = nx;
            target.y = ny;
        }
    }

    return target;
}

Point Robot::find_near(){
    cout << "find near  ";
    Point target(position);
    int min_step = board->floor[position.x][position.y].steps;
    bool target_cleaned = true;
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
        if (out_of_bound(nx, ny) || !board->floor[nx][ny].can_walk()){
            continue;
        }
        if (board->floor[nx][ny].steps < min_step){
            target.x = nx;
            target.y = ny;
            min_step = board->floor[nx][ny].steps;
            target_cleaned = board->floor[nx][ny].cleaned;
        }
        else if (board->floor[nx][ny].steps == min_step){
            if (target_cleaned && !board->floor[nx][ny].cleaned){
                target_cleaned = false;
                target.x = nx;
                target.y = ny;
                min_step = board->floor[nx][ny].steps;
            }
        }
    }
    return target;
}


Point Robot::get_closed_to_ldc(){
    /// TODO:
    cout << "finding " << last_dirty_cell.x << ", "
         << last_dirty_cell.y << endl;

    using std::abs;
    cout << "get close  ";
    int min_step = step_diff(position) + 2, min_dis = 9999;
    Point target(position);
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    for (int k = 0; k < 4; k++){
        Point next(position.x + dir[k][0], position.y + dir[k][1]);
        if (out_of_bound(next) || !board->floor[next.x][next.y].can_walk() ||
            board->floor[next.x][next.y].step_wrt_ldc < 0){
            continue;
        }

        int tmp1 = step_diff(next), tmp2 = last_dirty_cell.distance(next);
        if (tmp1 < min_step){
            min_step = tmp1;
            min_dis = tmp2;
            target = next;
        }
        else if (tmp1 == min_step && tmp2 < min_dis){
			min_step = tmp1;
            min_dis = tmp2;
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
            short lx = last_dirty_cell.x, ly = last_dirty_cell.y;
            if (board->floor[lx][ly].neighbor <= 0){
                find_dirty_cell();
            }

            plan_path_to_dirty();
            board->print_wrt_step_map();
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
        if (!out_of_bound(nx, ny) && board->floor[nx][ny].kind == '0'){
            board->floor[nx][ny].neighbor--;
        }
    }
}

void Robot::record_last_dirty(){
    const short dir[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    short min_dis = 9999;

    if (board->floor[position.x][position.y].neighbor > 1){
        last_dirty_cell = position;
    }
    else if (low_battery && board->floor[position.x][position.y].neighbor == 1){
        for (int k = 0; k < 4; k++){
            short nx = position.x + dir[k][0], ny = position.y + dir[k][1];
            if (out_of_bound(nx, ny) || !board->floor[nx][ny].need_to_clean()){
                continue;
            }
            if (board->floor[nx][ny].steps > board->floor[position.x][position.y].steps){
                last_dirty_cell = position;
            }
        }
    }

    /** for (int k = 0; k < 4; k++){
        Point next(position.x + dir[k][0], position.y + dir[k][1]);
        if (out_of_bound(next)){
            continue;
        }
        if (board->floor[next.x][next.y].need_to_clean() && 
            board->floor[next.x][next.y].steps < min_dis){
            last_dirty_cell = next;
            cout << "record (" << next.x << ", " << next.y << ")\n";
            min_dis = board->floor[next.x][next.y].steps;
        }
    } */
    
}

void Robot::find_dirty_cell(){
    cout << "find dirty  ";

    
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
            if (out_of_bound(next) || !board->floor[next.x][next.y].can_walk()){
                continue;
            }
            if (!visited[next.x][next.y]){
                que[front++] = next;
                visited[next.x][next.y] = true;
                if (!board->floor[next.x][next.y].cleaned){
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
    return;
}

void Robot::plan_path_to_dirty(){
    cout << "plan dirty to  (" << last_dirty_cell.x << ", " << last_dirty_cell.y << ")\n";
    int front = 0, tail = 0;
    const short dir[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
    que[front++] = last_dirty_cell;
    board->floor[last_dirty_cell.x][last_dirty_cell.y].step_wrt_ldc = 0;

    while (front > tail){
        Point curr = que[tail++];
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next) || !board->floor[next.x][next.y].can_walk()){
                continue;
            }
            if (board->floor[next.x][next.y].step_wrt_ldc == -1){  // not visited
                que[front++] = next;
                board->floor[next.x][next.y].step_wrt_ldc = 
                    board->floor[curr.x][curr.y].step_wrt_ldc + 1;

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
    board->floor[last_dirty_cell.x][last_dirty_cell.y].step_wrt_ldc = 0;

    while (front > tail){
        Point curr = que[tail++];
        board->floor[curr.x][curr.y].step_wrt_ldc = -1;
        for (int k = 0; k < 4; k++){
            Point next(curr.x + dir[k][0], curr.y + dir[k][1]);
            if (out_of_bound(next) || !board->floor[next.x][next.y].can_walk()){
                continue;
            }
            if (board->floor[next.x][next.y].step_wrt_ldc != -1){  // not visited
                que[front++] = next;
            }
        }
    }
}

Point Robot::sweep_one_cell(){
    // cout << "sweep" << endl;
    position = pick_one_cell();
    if (position == last_dirty_cell){
        reset_path();
        finding_path = false;
    }
    if (!board->floor[position.x][position.y].cleaned){
        board->floor[position.x][position.y].cleaned = true;
        cells_to_clean--;
        decrease_neighbor();
    }
    if (!finding_path){
        record_last_dirty();
    }

    //cout << "pick finish" << endl;

    battery--;
    if (position == charger){
        battery = full_bat;
        low_battery = false;
        if (finding_path){
            reset_path();
            plan_path_to_dirty();
        }
        cout << "charge\n";
    }
    if (battery <= 0){
        throw "out of power";
    }

    return position;
}

#endif