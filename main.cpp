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

8 8 30
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

7 15 100
111111111111111
110110000011101
110000000001101
111111101111001
1000000R0000001
110001011001001
111111111111111

6 6 10
111111
100001
10R001
100001
100001
111111

10 12 50
111111111111
100000000001
111110000011
100000111011
111000111011
111111000001
110000111001
111000110001
111100000011
11111111R111

10 15 100
111111111111111
111000100111001
100001110010001
100110001001001
110011001011001
100010001001101
100011000011001
110100110100011
110000000001111
11111111R111111

*/