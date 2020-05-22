/*
 *  Orca-Components: Components for robotics.
 *
 *  Copyright (C) 2004
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "Worker.h"

#define GRIDMAP_DEBUG

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef M_2PI
    #define M_2PI (2.0*M_PI)
#endif

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#define DTOR(d) ((d) * M_PI / 180)
#define RTOD(r) ((r*180.0)/M_PI)

typedef struct GridMapParams{
    double cell_size;
    double map_size;
    double pmax;
    double pmin;
    double laser_max_range;
    double laser_prec;
    double laser_sigma;
    double laser_plambda;
    double laser_weight_hit;
    double laser_weight_unexpected;
}GridMapParams_t;

typedef struct Pose{
    double x;
    double y;
    double th;
}Pose_t;

typedef struct Point_t{
    double x;
    double y;
}Point_t;

typedef struct Cell_t{
    int x;
    int y;
    double log_ratio;
}Cell_t;

typedef struct Arc{
    double r;
    double th;
    double beam_angle;
}Arc_t;

class GridMap
{
public:
    GridMap(GridMapParams_t &gridmapParams, bool showMap);
    ~GridMap();
    void Init();
    void UpdateLaser(std::vector<Point_t > &laser, Pose_t &robot);
    void ResetMap();
    std::vector<Point_t> GetMapPoints(double lmap_threshold);
    std::vector<Point_t> GetLocalMapPoints(double lmap_threshold);
    //std::vector<std::vector<double> > map;
    CImg<double> *map;
    CImg<double> *map_inverted;
    std::vector<std::vector<double> > local_map;
    static double logRatio2Prob(double log_ratio);
    Point_t cell2Point(int i, int j);
    void saveMap(std::string str);
    int loadMap(std::string str);
    Worker *worker;
private:
    GridMapParams_t _gridmapParams;
    unsigned int cells;
    unsigned int local_cells;
    int center;
    int local_center;
    unsigned int laser_size;
    double lmax;
    double lmin;
    double lmap_threshold;
    bool _showMap;
    std::vector<std::vector<double> > local_map_dist;
    std::vector<std::vector<double> > local_map_ang;
    std::vector<std::vector<std::vector<Cell_t> > > local_map_beams;
    double LaserLogRatio(double z, double zm);
};

#endif
