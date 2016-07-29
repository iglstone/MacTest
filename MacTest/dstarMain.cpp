////
////  main.cpp
////  DStarPathPlaner
////
////  Created by 郭龙 on 16/7/4.
////  Copyright © 2016年 690193240@qq.com. All rights reserved.
////
//
//#include <stdio.h>
//#include "Dstar.h"
//#include <stdlib.h>
//#include <iostream>
//using namespace std;
//
//vector<double *>listToVector(list<state> list2){
//    vector<double *> vec;
//    list<state>::iterator iter;
//    //进行迭代遍历
//    for(iter = list2.begin(); iter != list2.end(); iter++)
//    {
//        double * v = new double[2];
//        v[0] = iter->x;
//        v[1] = iter->y;
//        vec.push_back(v);
//    }
//    return vec;
//}
//
//void mapCost(unsigned char *map, int width, int height , Dstar *star){
//    for (int i = 0; i<height; i++) {
//        for (int j = 0; j<width; j++) {
//            if (map[i*width + j] == 255) {
//                star -> updateCell(j, i, -1);
//            }
//        }
//    }
//}
//
//int main() {
//    Dstar *dstar = new Dstar();
//    //list<state> mypath;
//    ds_path mypath;
//    
//    dstar->init(0,0,10,5);         // set start to (0,0) and goal to (10,5)
//    dstar->updateCell(3,4,-1);     // set cell (3,4) to be non traversable
//    dstar->updateCell(2,2,42.432); // set set (2,2) to have cost 42.432
//    
//    dstar->replan();               // plan a path
//    mypath = dstar->getPath();     // retrieve path
//    
//    vector<double *> vec ;
//    vec = listToVector(mypath.path);
//    
//    for (int k=0; k<(int)vec.size(); ++k)
//    {
//        double * v = vec[k];
//        int x = v[0];
//        int y = v[1];
//        delete v;
//        printf("%d:%d_%d\n",k, x, y);
//        //newMaps[x + mapSize*y] = 255;
//    }
//    
//    dstar->updateStart(10,2);      // move start to (10,2)
//    dstar->replan();               // plan a path
//    mypath = dstar->getPath();     // retrieve path
//    
//    dstar->updateGoal(0,1);        // move goal to (0,1)
//    dstar->replan();               // plan a path
//    mypath = dstar->getPath();     // retrieve path
//    
//    return 0;
//}
//
