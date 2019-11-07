#include <iostream>
#include "board.hpp"
#include "robot.hpp"

using namespace std;

int main(){
    int rows, cols, battery;
    cin >> rows >> cols >> battery;
    Board floorplan(rows, cols);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++){
            cin >> floorplan.floor[i][j].kind;
        }
    }

    Robot bot(battery, &floorplan);

    


    return 0;
}