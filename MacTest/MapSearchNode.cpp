//
//  MapSearchNode.cpp
//  AStarPathFinding
//
//  Created by 郭龙 on 16/7/1.
//  Copyright © 2016年 690193240@qq.com. All rights reserved.
//

#include "MapSearchNode.hpp"
#include <math.h>

// map helper functions
MapSearchNode::MapSearchNode( unsigned char *map ,int thresh ){
    
    x = y = 0;
    
    memcpy(world_map, map, sizeof(unsigned char)*MAP_WIDTH*MAP_HEIGHT);
    
    THRSHHOLD_MAXVALUE = thresh;
    
    map2 = map;
}

//MapSearchNode::MapSearchNode( int *map ,int thresh){
//    x = y = 0;
//    memcpy(world_map, map, sizeof(int)*MAP_WIDTH*MAP_HEIGHT);
//    
//    THRSHHOLD_MAXVALUE = thresh;
//}

//int MapSearchNode::GetMap( int x, int y )
unsigned char MapSearchNode::GetMap( int x, int y )
{
    if( x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT )
    {
        return THRSHHOLD_MAXVALUE;
    }
    
    unsigned char t = world_map[(y*MAP_WIDTH)+x];
    printf("%d_",t);
    return t;//world_map[(y*MAP_WIDTH)+x];
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
    
    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
       (y == rhs.y) )
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    
    sprintf( str, "Node position : (%d,%d,%d)\n", x,y,world_map[y*MAP_WIDTH + x] );
    
    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
    
    if( (x == nodeGoal.x) &&
       (y == nodeGoal.y) )
    {
        return true;
    }
    
    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
    
    int parent_x = -1;
    int parent_y = -1;
    
    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }
    
    MapSearchNode NewNode;// = MapSearchNode(world_map);

    // push each possible move except allowing the search to go backwards
    
    if( (GetMap( x-1, y ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x-1) && (parent_y == y))
       )
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }
    
    if( (GetMap( x, y-1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x) && (parent_y == y-1))
       )
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    if( (GetMap( x+1, y ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x+1) && (parent_y == y))
       )
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }
    
    
    if( (GetMap( x, y+1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x) && (parent_y == y+1))
       )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    /**
     *  add by guolong
     */
    
    
    if( (GetMap( x+1, y+1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x+1) && (parent_y == y+1))
       )
    {
        NewNode = MapSearchNode( x+1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    if( (GetMap( x-1, y+1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x-1) && (parent_y == y+1))
       )
    {
        NewNode = MapSearchNode( x-1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    if( (GetMap( x+1, y-1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x+1) && (parent_y == y-1))
       )
    {
        NewNode = MapSearchNode( x+1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    if( (GetMap( x-1, y-1 ) < THRSHHOLD_MAXVALUE)
       && !((parent_x == x-1) && (parent_y == y-1))
       )
    {
        NewNode = MapSearchNode( x-1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }
    
    
    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    return (float) GetMap( x, y );
}

vector<double *> MapSearchNode::StartSearching(int x0, int y0, int x1, int y1){
    cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";
    
    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost
    // of travel across the terrain. Zero means the least possible difficulty
    // in travelling (think ice rink if you can skate) whilst 5 represents the
    // most difficult. 9 indicates that we cannot pass.
    
    // Create an instance of the search class...
    
    AStarSearch<MapSearchNode> astarsearch;
    unsigned int SearchCount = 0;
    const unsigned int NumSearches = 1;
    vector<double *> path;
    
    while(SearchCount < NumSearches)
    {
        
        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.x = x0;// rand()%MAP_WIDTH;
        nodeStart.y = y0;//rand()%MAP_HEIGHT;
        
        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.x = x1;//rand()%MAP_WIDTH;
        nodeEnd.y = y1;//rand()%MAP_HEIGHT;
        
        // Set Start and goal states
        
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
        
        unsigned int SearchState;
        unsigned int SearchSteps = 0;
        
        do
        {
            SearchState = astarsearch.SearchStep();
            SearchSteps++;
        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
        
        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            cout << "Search found goal state\n";
            
            MapSearchNode *node = astarsearch.GetSolutionStart();
            
            int steps = 0;
            
            node->PrintNodeInfo();
            
            for( ;; )
            {
                node = astarsearch.GetSolutionNext();
                if( !node )
                {
                    break;
                }
                node->PrintNodeInfo();
                
                double * v = new double[2];
                v[0] = node->x;
                v[1] = node->y;
                path.push_back(v);
                
                steps ++;
            };
            
            cout << "Solution steps " << steps << endl;
            
            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();
        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
        {
            cout << "Search terminated. Did not find goal state\n";
        }
        
        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";
        
        SearchCount ++;
        astarsearch.EnsureMemoryFreed();
    }
    return path;
}

