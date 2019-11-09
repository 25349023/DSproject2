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
    cout << endl << endl;

    int count = 0;
    cout << charger.x << " " << charger.y << endl;
    while (!bot.is_finished()){
        Point p = bot.sweep_one_cell();
        cout << p.x << " " << p.y << endl;
        count++;
    }
    cout << "steps: " << count << endl;


    return 0;
}

/*
8 6 20
111111
100011
100001
100001
110001
110011
110011
111R11

8 8 300
11111111
11000111
10000101
10000001
11100001
10001101
10000001
11111R11

7 10 20
1111111111
1000111111
1000111111
1000111111
1011111111
1000011111
1111R11111
*/