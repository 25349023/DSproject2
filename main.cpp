#include <iostream>
#include "board.hpp"
#include "robot.hpp"

using namespace std;

int main(){
    int rows, cols, battery;
    cin >> rows >> cols >> battery;
    Board floorplan(rows, cols);
    Point charger;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++){
            cin >> floorplan.floor[i][j].kind;
            if (floorplan.floor[i][j].kind == 'R'){
                charger.x = i;
                charger.y = j;
            }
        }
    }

    Robot bot(battery, charger, &floorplan);
    floorplan.print_step_map();
    cout << endl;
    floorplan.print_neighbor_map();
    cout << endl;

    //while (!bot.is_finished()){
      //  Point p = bot.pick_one_cell();
        //cout << p.x << " " << p.y << endl;
    //}


    return 0;
}