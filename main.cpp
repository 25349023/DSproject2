#include <iostream>
#include <fstream>
#include <cstdio>
#include "board.hpp"
#include "robot.hpp"

using namespace std;

int main(){
    ifstream fin("floor.data");
    if (!fin){
        cerr << "error on opening floor.data";
        return 1;
    }

    int rows, cols, battery;
    fin >> rows >> cols >> battery;
    Board floorplan(rows, cols);
    Point charger;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++){
            fin >> floorplan[i][j].kind;
            if (floorplan[i][j].kind == 'R'){
                charger.x = i;
                charger.y = j;
            }
        }
    }
    fin.close();

    Robot bot(battery, charger, &floorplan);

    int count = 0;
    FILE *tmpf = tmpfile();
    fprintf(tmpf, "%hd %hd\n", charger.x, charger.y);
    while (!bot.is_finished()){
        Point p = bot.sweep_one_cell();
        fprintf(tmpf, "%hd %hd\n", p.x, p.y);
        count++;
    }

    char line[256];
    ofstream fout("final.path");
    if (!fout){
        cerr << "error on opening final.path";
        return 1;
    }
    fout << count << endl;
    rewind(tmpf);
    while (!feof(tmpf)){
        if (fgets(line, sizeof line, tmpf) != NULL){
            fout << line;
        }
    }
    fout.close();
    fclose(tmpf);

    return 0;
}
