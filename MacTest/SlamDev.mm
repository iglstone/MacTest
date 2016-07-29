//
//  SlamDev.m
//  MacTest
//
//  Created by 郭龙 on 16/7/29.
//  Copyright © 2016年 郭龙. All rights reserved.
//

// SinglePositionSLAM params: gives us a nice-size map
static const int MAP_SIZE_PIXELS        = 800;
static const double MAP_SIZE_METERS     =  32;
static const int SCAN_SIZE 		        = 682;

// Arbitrary maximum length of line in input logfile
#define MAXLINE 10000

#include <iostream>
#include <vector>
using namespace std;

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "Position.hpp"
#include "Laser.hpp"
#include "WheeledRobot.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"
#include "MapSearchNode.hpp"
#include "Dstar.h"
#import "SlamDev.h"

// Methods to load all data from file ------------------------------------------
// Each line in the file has the format:
//
//  TIMESTAMP  ... Q1  Q1 ... Distances
//  (usec)                    (mm)
//  0          ... 2   3  ... 24 ...
//
//where Q1, Q2 are odometry values

static void skiptok(char ** cpp)
{
    *cpp = strtok(NULL, " ");
}

static int nextint(char ** cpp)
{
    skiptok(cpp);
    return atoi(*cpp);
}

static void load_data(
                      const char * dataset,
                      vector<int *> & scans,
                      vector<long *> & odometries)
{
    char filename[256];
    
    sprintf(filename, "%s.dat", dataset);
    printf("Loading data from %s ... \n", filename);
    
    FILE * fp = fopen(filename, "rt");
    
    if (!fp)
    {
        fprintf(stderr, "Failed to open file\n");
        exit(1);
    }
    
    char s[MAXLINE];
    
    while (fgets(s, MAXLINE, fp))
    {
        char * cp = strtok(s, " ");
        
        long * odometry = new long [3];
        odometry[0] = atol(cp);
        skiptok(&cp);
        odometry[1] = nextint(&cp);
        odometry[2] = nextint(&cp);
        
        odometries.push_back(odometry);
        
        // Skip unused fields
        for (int k=0; k<20; ++k)
        {
            skiptok(&cp);
        }
        
        int * scanvals = new int [SCAN_SIZE];
        
        for (int k=0; k<SCAN_SIZE; ++k)
        {
            scanvals[k] = nextint(&cp);
        }
        
        scans.push_back(scanvals);
    }
    
    fclose(fp);    
}

// Class for Mines verison of URG-04LX Lidar -----------------------------------

class MinesURG04LX : public URG04LX
{
public:
    
    MinesURG04LX(void): URG04LX(
                                70,          // detectionMargin
                                145)         // offsetMillimeters
    {
    }
};

// Class for MinesRover custom robot -------------------------------------------

class Rover : WheeledRobot
{
public:
    Rover() : WheeledRobot(
                           77,     // wheelRadiusMillimeters
                           165)     // halfAxleLengthMillimeters
    {
    }

    Velocities computeVelocities(long * odometry, Velocities & velocities)
    {
        return WheeledRobot::computeVelocities(
                                               odometry[0],
                                               odometry[1],
                                               odometry[2]);
    }

protected:
    void extractOdometry(
                         double timestamp,
                         double leftWheelOdometry,
                         double rightWheelOdometry,
                         double & timestampSeconds,
                         double & leftWheelDegrees,
                         double & rightWheelDegrees)
    {
        // Convert microseconds to seconds, ticks to angles
    timestampSeconds                 = timestamp / 1e6;
    leftWheelDegrees                 = ticksToDegrees(leftWheelOdometry);
    rightWheelDegrees                = ticksToDegrees(rightWheelOdometry);
    }
    
    void descriptorString(char * str)
    {
        sprintf(str, "ticks_per_cycle=%d", this->TICKS_PER_CYCLE);
    }

private:

    double ticksToDegrees(double ticks)
    {
        return ticks * (180. / this->TICKS_PER_CYCLE);
    }

    static const int TICKS_PER_CYCLE = 2000;
};


// Progress-bar class
// Adapted from http://code.activestate.com/recipes/168639-progress-bar-class/
// Downloaded 12 January 2014

class ProgressBar
{
public:
    
    ProgressBar(int minValue, int maxValue, int totalWidth)
    {
        strcpy(this->progBar, "[]");   // This holds the progress bar string
        this->min = minValue;
        this->max = maxValue;
        this->span = maxValue - minValue;
        this->width = totalWidth;
        this->amount = 0;       // When amount == max, we are 100% done
        this->updateAmount(0);  // Build progress bar string
    }
    
    void updateAmount(int newAmount)
    {
        if (newAmount < this->min)
        {
            newAmount = this->min;
        }
        if (newAmount > this->max)
        {
            newAmount = this->max;
        }
        
        this->amount = newAmount;
        
        // Figure out the new percent done, round to an integer
        float diffFromMin = float(this->amount - this->min);
        int percentDone = (int)round((diffFromMin / float(this->span)) * 100.0);
        
        // Figure out how many hash bars the percentage should be
        int allFull = this->width - 2;
        int numHashes = (int)round((percentDone / 100.0) * allFull);
        
        // Build a progress bar with hashes and spaces
        strcpy(this->progBar, "[");
        this->addToProgBar("#", numHashes);
        this->addToProgBar(" ", allFull-numHashes);
        strcat(this->progBar, "]");
        
        // Figure out where to put the percentage, roughly centered
        int percentPlace =  (strlen(this->progBar) / 2) - ((log10(percentDone+1)) + 1);
        char percentString[5];
        sprintf(percentString, "%d%%", percentDone);
        
        // Put it there
        for (int k=0; k<strlen(percentString); ++k)
        {
            this->progBar[percentPlace+k] = percentString[k];
        }
        
    }
    
    char * str()
    {
        return this->progBar;
    }
    
private:
    
    char progBar[1000]; // more than we should ever need
    int min;
    int max;
    int span;
    int width;
    int amount;
    
    void addToProgBar(const char * s, int n)
    {
        for (int k=0; k<n; ++k)
        {
            strcat(this->progBar, s);
        }
    }
};

// Helpers ----------------------------------------------------------------

int coords2index(double x,  double y)
{
    return y * MAP_SIZE_PIXELS + x;
}

vector<double *>listToVector(list<state> list2)
{
    vector<double *> vec;
    list<state>::iterator iter;
    //进行迭代遍历
    for(iter = list2.begin(); iter != list2.end(); iter++)
    {
        double * v = new double[2];
        v[0] = iter->x;
        v[1] = iter->y;
        vec.push_back(v);
    }
    return vec;
}

void mapCost(unsigned char *map, int width, int height , Dstar *star)
{
    for (int i = 0; i<height; i++) {
        for (int j = 0; j<width; j++) {
            if (map[i*width + j] == 188) {
                star -> updateCell(j, i, -1);
            }
        }
    }
}

int mm2pix(double mm)
{
    return (int)(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));
}




@implementation SlamDev
- (instancetype)init
{
    self = [super init];
    if (self) {
        return self;
    }
    return nil;
}

- (void) slamMap
{
    const char * dataset = "/Users/guolong/Desktop/exp2";//argv[1];
    bool use_odometry    = false; //atoi(argv[2]) ? true : false;
    int random_seed = 9999;// argc > 3 ? atoi(argv[3]) : 0;
    
    // Load the Lidar and odometry data from the file
    vector<int *> scans;
    vector<long *> odometries;
    load_data(dataset, scans, odometries);
    
    // Build a robot model in case we want odometry
    Rover robot = Rover();
    
    // Create a byte array to receive the computed maps
    unsigned char * mapbytes = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];
    
    // Create SLAM object
    MinesURG04LX laser;
    SinglePositionSLAM * slam = random_seed ?
    (SinglePositionSLAM*)new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed) :
    (SinglePositionSLAM*)new Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);
    
    // Report what we're doing
    int nscans = (int) scans.size();
    printf("Processing %d scans with%s odometry / with%s particle filter...\n",
           nscans, use_odometry ? "" : "out", random_seed ? "" : "out");
    ProgressBar * progbar = new ProgressBar(0, nscans, 80);
    
    // Start with an empty trajectory of positions
    vector<double *> trajectory;
    
    // Start timing
    time_t start_sec = time(NULL);
    
    // Loop over scans
    for (int scanno=0; scanno < nscans; ++scanno)
    {
        int * lidar = scans[scanno];
        
        // Update with/out odometry
        if (use_odometry)
        {
            Velocities velocities = robot.computeVelocities(odometries[scanno], velocities);
            slam->update(lidar, velocities);
        }
        else
        {
            slam->update(lidar);
        }
        
        Position position = slam->getpos();
        
        // Add new coordinates to trajectory
        double * v = new double[2];
        v[0] = position.x_mm;
        v[1] = position.y_mm;
        trajectory.push_back(v);
        
        // Tame impatience
        progbar->updateAmount(scanno);
        printf("\r%s", progbar->str());
        fflush(stdout);
    }
    
    // Report speed
    time_t elapsed_sec = time(NULL) - start_sec;
    printf("\n%d scans in %ld seconds = %f scans / sec\n",
           nscans, elapsed_sec, (float)nscans/elapsed_sec);
    
    // Get final map
    slam->getmap(mapbytes);
    
    // Put trajectory into map as black pixels
    for (int k=0; k<(int)trajectory.size(); ++k)
    {
        double * v = trajectory[k];
        
        int x = mm2pix(v[0]);
        int y = mm2pix(v[1]);
        
        delete v;
        
        mapbytes[coords2index(x, y)] = 0;
    }
    
    //grid map
    int gridSize = 5;
    int mapSize = MAP_SIZE_PIXELS/gridSize;
    unsigned char * newMaps = new unsigned char[mapSize * mapSize];
    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            //判断10x10单元格里有没有黑色的点
            int sum = 0;
            for (int t1 = 0; t1 < gridSize; t1++) {
                for (int t2= 0; t2 < gridSize; t2++) {
                    unsigned char m = mapbytes[coords2index(t1+i*gridSize, t2+j*gridSize)];
                    sum += m;
                }
            }
            
            int average = sum / (gridSize*gridSize);
            if (average > 178) {//127,100,110,117,120,125,126,127,128,138,148,188
                newMaps[i+j*mapSize] = 188;
            }else
                newMaps[i+j*mapSize] = 0;
        }
    }
    printf("S:%d\n",newMaps[70+80*mapSize]);
    printf("E:%d\n",newMaps[30+50*mapSize]);
    
    //mapSize = 20;
    //D* path finding
    Dstar *dstar = new Dstar();
    ds_path mypath;
    dstar->init(0,0,mapSize-1,mapSize-1);         // set start to (0,0) and goal to (10,5)
    //mapCost(newMaps, mapSize, mapSize, dstar);
    
    for (int i = 0; i<mapSize; i++) {
        for (int j = 0; j<mapSize; j++) {
            if (newMaps[i*mapSize + j] == 188) {
                dstar -> updateCell(j, i, -1);
            }
        }
    }
    
    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path
    
    dstar->updateGoal(100, 130);
    dstar->replan();
    mypath = dstar->getPath();
    
    dstar->updateStart(0,10);      // move start to (10,2)
    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path
    
    //dstar->updateGoal(10,20);        // move goal to (0,1)
    //dstar->replan();               // plan a path
    //mypath = dstar->getPath();     // retrieve path
    
    vector<double *> vec ;
    vec = listToVector(mypath.path);
    for (int k=0; k<(int)vec.size(); ++k)
    {
        double * v = vec[k];
        int x = v[0];
        int y = v[1];
        delete v;
        printf("%d:%d_%d\n",k, x, y);
        newMaps[x + mapSize*y] = 255;
    }
    
    /*
     // A* path algrithm search map
     vector<double *> path;
     MapSearchNode mapSearch = MapSearchNode(newMaps, 129);
     path = mapSearch.StartSearching( 0, 10, 100, mapSize - 20 );//( 70, 80, 40, 40 );
     if (path.size() == 0) {
     printf("....zero path num");
     } else {
     printf("path num: %lu", path.size());
     for (int k=0; k<(int)path.size(); ++k)
     {
     double * v = path[k];
     int x = v[0];
     int y = v[1];
     delete v;
     
     newMaps[x + mapSize*y] = 255;
     }
     }
     */
    
    //save grid map
    char filename2[100];
    sprintf(filename2, "%s.pgm", "/Users/guolong/Desktop/shange");
    printf("\nSaving map to file %s\n", filename2);
    FILE * output2 = fopen(filename2, "wt");
    fprintf(output2, "P2\n%d %d 255\n", mapSize, mapSize);
    for (int y=0; y<mapSize; y++)
    {
        for (int x=0; x<mapSize; x++)
        {
            fprintf(output2, "%d ", newMaps[x+y*mapSize]);
        }
        fprintf(output2, "\n");
    }
    printf("\n");
    
    
    /*
     //A* searching the shortest path
     vector<double *> path;
     MapSearchNode mapSearch = MapSearchNode(mapbytes, 128);
     path = mapSearch.StartSearching( 180, 400, 200, 320 );
     if (path.size() == 0) {
     printf("....zero path num");
     } else {
     printf("path num: %lu", path.size());
     for (int k=0; k<(int)path.size(); ++k)
     {
     double * v = path[k];
     int x = v[0];
     int y = v[1];
     delete v;
     mapbytes[coords2index(x, y)] = 0;
     }
     }
     */
    
    
    // Save map and trajectory as PGM file
    
    char filename[100];
    sprintf(filename, "%s.pgm", dataset);
    printf("\nSaving map to file %s\n", filename);
    
    FILE * output = fopen(filename, "wt");
    
    fprintf(output, "P2\n%d %d 255\n", MAP_SIZE_PIXELS, MAP_SIZE_PIXELS);
    
    for (int y=0; y<MAP_SIZE_PIXELS; y++)
    {
        for (int x=0; x<MAP_SIZE_PIXELS; x++)
        {
            fprintf(output, "%d ", mapbytes[coords2index(x, y)]);
        }
        fprintf(output, "\n");
    }
    
    printf("\n");
    
    // Clean up
    for (int scanno=0; scanno<(int)scans.size(); ++scanno)
    {
        delete scans[scanno];
        delete odometries[scanno];
    }
    
    if (random_seed)
    {
        delete ((RMHC_SLAM *)slam);
    }
    else
    {
        delete ((Deterministic_SLAM *)slam);
    }
    
    delete progbar;
    delete[] mapbytes;
    fclose(output);
    
}

@end
