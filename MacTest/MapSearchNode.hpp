//
//  MapSearchNode.hpp
//  AStarPathFinding
//
//  Created by 郭龙 on 16/7/1.
//  Copyright © 2016年 690193240@qq.com. All rights reserved.
//

#ifndef MapSearchNode_hpp
#define MapSearchNode_hpp

#include <stdio.h>
#include "stlastar.h"
using namespace std;

const int MAP_WIDTH = 800/5;
const int MAP_HEIGHT = 800/5;
static int THRSHHOLD_MAXVALUE ;//125
//static int world_map[ MAP_WIDTH * MAP_HEIGHT ];
static unsigned char world_map[ MAP_WIDTH * MAP_HEIGHT ];
static unsigned char *map2;

class MapSearchNode
{
public:
    int x;	 // the (x,y) positions of the node
    int y;
    
    MapSearchNode() { x = y = 0; };
    MapSearchNode( int px, int py ) { x=px; y=py; };
//    MapSearchNode( int *map ,int thresh );
    MapSearchNode( unsigned char *map ,int thresh );
    
    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );
    
    void PrintNodeInfo();
    unsigned char GetMap( int x, int y );
//    void StartSearching(int x0, int y0, int x1, int y1);//x0 start , x1 end
    vector<double *> StartSearching(int x0, int y0, int x1, int y1);//x0 start , x1 end
    
    /**
     *  demo
     MapSearchNode mapsearch = MapSearchNode( int *map ,int thresh );
     mapsearch.StartSearching(int x0, int y0, int x1, int y1);//x0 start , x1 end
     */
    
};

#endif /* MapSearchNode_hpp */
