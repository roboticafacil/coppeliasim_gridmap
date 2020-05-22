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

#include "gridmap/gridmap.h"

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <iostream>

GridMap::GridMap(GridMapParams_t &gridmapParams, bool showMap): map(NULL), map_inverted(NULL), _gridmapParams(gridmapParams), _showMap(showMap), worker(NULL)
{
}

/**
 * Class destructor
 */
GridMap::~GridMap()
{
    local_map.clear();
    local_map_dist.clear();
    local_map_ang.clear();
    local_map_beams.clear();
    if (worker)
        delete worker;
    if (map)
        delete map;
    if (map_inverted)
        delete map_inverted;
}

void GridMap::Init()
{
    int x,y;
    double dx,dy;
    cells=(unsigned int)(floor(_gridmapParams.map_size/_gridmapParams.cell_size));
    map = new CImg<double>(cells,cells,1);
    map_inverted = new CImg<double>(cells,cells,1);
    local_map.clear();
    local_map_dist.clear();
    local_map_ang.clear();
    local_map_beams.clear();
    local_cells=(unsigned int)(floor(2.0*_gridmapParams.laser_max_range/_gridmapParams.cell_size));
    laser_size=(unsigned int)(floor(_gridmapParams.laser_max_range/_gridmapParams.laser_prec));
    lmax=log(_gridmapParams.pmax/(1.0-_gridmapParams.pmax));
    lmin=log(_gridmapParams.pmin/(1.0-_gridmapParams.pmin));
    center=cells/2;
    std::vector<double> local_row(local_cells);
    std::vector<std::vector<Cell_t> > local_cells_row;
    std::vector<Cell_t> cells_bucket;
    local_cells_row.clear();
    cells_bucket.clear();
    Cell_t cell;
    for (unsigned int i=0;i<local_cells;i++)
    {
        local_row[i]=0.0;
        local_cells_row.push_back(cells_bucket);
    }
    for (unsigned int i=0;i<local_cells;i++)
    {
        local_map_dist.push_back(local_row);
        local_map_ang.push_back(local_row);
        local_map.push_back(local_row);
        local_map_beams.push_back(local_cells_row);
    }
    local_center=local_cells/2;
    for (int i=0;i<local_cells;i++)
    {
        for (int j=0;j<local_cells;j++)
        {
            x=i-local_center;
            y=j-local_center;
            dx=double(x)*_gridmapParams.cell_size;
            dy=double(y)*_gridmapParams.cell_size;
            local_map_dist[i][j]=sqrt(dx*dx+dy*dy);
            local_map_ang[i][j]=atan2(dy,dx);
        }
    }
    for (int i=0;i<local_cells;i++)
    {
        for (int j=0;j<local_cells;j++)
        {
            double zm=local_map_dist[i][j];
            double th=local_map_ang[i][j];
            double cth,sth;
            cth=cos(th);
            sth=sin(th);
            for (int k=0;k<laser_size;k++)
            {
                double z;
                z=double(k)*_gridmapParams.laser_prec;
                cell.x=int(floor((z*cth)/_gridmapParams.cell_size+local_center));
                cell.y=int(floor((z*sth)/_gridmapParams.cell_size+local_center));
                if ((cell.x>=0)&&(cell.y>=0)&&(cell.x<local_cells)&&(cell.y<local_cells))
                {
                    cell.log_ratio=LaserLogRatio(z,zm);
                    local_map_beams[i][j].push_back(cell);
                }
            }
        }
    }
    if (_showMap)
    {
        worker = new Worker(map_inverted);
        worker->start();
    }
}

void GridMap::UpdateLaser(std::vector<Point_t > &laser, Pose_t &robot)
{
    int local_x,local_y;
    int robot_x,robot_y;
    double cth,sth;
    cth=cos(-robot.th);
    sth=sin(-robot.th);
    robot_x=floor(robot.x/_gridmapParams.cell_size);
    robot_y=floor(robot.y/_gridmapParams.cell_size);
    for (int i=0;i<local_cells;i++)
    {
        for (int j=0;j<local_cells;j++)
            local_map[i][j]=0.0;
    }
    for (unsigned int i=0;i<laser.size();i++)
    {
        local_x=(int)(floor((cth*laser[i].x+sth*laser[i].y)/_gridmapParams.cell_size+local_center));
        local_y=(int)(floor((sth*laser[i].x-cth*laser[i].y)/_gridmapParams.cell_size+local_center));
        if ((local_x>=0)&&(local_y>=0)&&(local_x<local_cells)&&(local_y<local_cells))
        {
            std::vector<Cell_t>::iterator it_end=local_map_beams[local_x][local_y].end();
            for (std::vector<Cell_t>::iterator it=local_map_beams[local_x][local_y].begin();it!=it_end;++it)
            {
                local_map[it->x][it->y]+=it->log_ratio;
                if (local_map[it->x][it->y]>lmax)
                    local_map[it->x][it->y]=lmax;
                if (local_map[it->x][it->y]<lmin)
                    local_map[it->x][it->y]=lmin;
            }
        }
    }
    if (_showMap)
        worker->mutex.lock();
    for (int i=0;i<local_cells;i++)
    {
        for (int j=0;j<local_cells;j++)
        {
            local_x=i+center-local_center+robot_x;
            local_y=j+center-local_center-robot_y;
            if ((local_x>=0)&&(local_y>=0)&&(local_x<cells)&&(local_y<cells))
            {
                CImg<double>::iterator it = map->begin()+local_y*cells+local_x;
                CImg<double>::iterator it1 = map_inverted->begin()+local_y*cells+local_x;
                *it=(*it)+local_map[i][j];
                if ((*it)>lmax)
                    *it=lmax;
                if ((*it)<lmin)
                    *it=lmin;
                *it1=-(*it);
            }
        }
    }
    if (_showMap)
        worker->mutex.unlock();
}

void GridMap::ResetMap()
{
    if (_showMap)
        worker->mutex.lock();
    map->fill(0.0);
    map_inverted->fill(0.0);
    if (_showMap)
        worker->mutex.unlock();
}

Point_t GridMap::cell2Point(int i, int j)
{
    Point_t p;
    p.x=double(i-center)*_gridmapParams.cell_size;
    p.y=double(j-center)*_gridmapParams.cell_size;
    return p;
}

std::vector<Point_t> GridMap::GetMapPoints(double lmap_threshold)
{
    std::vector<Point_t> points;
    points.clear();
    Point_t p;
    for (int i=0;i<cells;i++)
    {
        for (int j=0;j<cells;j++)
        {
            if (map[i][j]>=lmap_threshold)
            {
                p.x=double(i-center)*_gridmapParams.cell_size;
                p.y=double(j-center)*_gridmapParams.cell_size;
                points.push_back(p);
            }
        }
    }
    return points;
}

std::vector<Point_t> GridMap::GetLocalMapPoints(double lmap_threshold)
{
    std::vector<Point_t> points;
    points.clear();
    Point_t p;
    for (int i=0;i<local_cells;i++)
    {
        for (int j=0;j<local_cells;j++)
        {
            if (local_map[i][j]>=lmap_threshold)
            {
                p.x=double(i-local_center)*_gridmapParams.cell_size;
                p.y=double(j-local_center)*_gridmapParams.cell_size;
                points.push_back(p);
            }
        }
    }
    return points;
}

double GridMap::LaserLogRatio(double z, double zm)
{
    double phit,punexpected,p,l,lambda;
    lambda=-log(0.5/_gridmapParams.laser_plambda)/zm;
    phit=_gridmapParams.pmax*exp(-0.5*((z-zm)/_gridmapParams.laser_sigma)*((z-zm)/_gridmapParams.laser_sigma));
    if (z<=zm)
        phit=fmax(phit,_gridmapParams.pmin);
    else
        phit=fmax(phit,0.5);
    if (z<zm)
        punexpected=_gridmapParams.laser_plambda*exp(-lambda*z);
    else
        punexpected=0.5;
    p=_gridmapParams.laser_weight_hit*phit+_gridmapParams.laser_weight_unexpected*punexpected;
    l=log(p/(1-p));
    l=fmin(fmax(l,lmin),lmax);
    return l;
}

double GridMap::logRatio2Prob(double log_ratio)
{
    return exp(log_ratio)/(1.0+exp(log_ratio));
}

void GridMap::saveMap(std::string str)
{
    if (worker)
        worker->mutex.lock();
    CImg<double> im(*map_inverted);
    if (worker)
        worker->mutex.unlock();
    im-=lmin;
    im*=(255.0/(lmax-lmin));
    CImg<unsigned char> im1(im);
    im1.save(str.c_str());
}

int GridMap::loadMap(std::string str)
{
    CImg<unsigned char> im(str.c_str());
    if ((im.width()==map->width())&&(im.height()==map->height()))
    {
        CImg<double>gray(im.channel(0));
        gray*=((lmax-lmin)/255.0);
        gray+=lmin;
        CImg<double>::iterator itGray=gray.begin();
        if (worker)
            worker->mutex.lock();
        *map_inverted=gray;
        *map=-(*map_inverted);
        if (worker)
            worker->mutex.unlock();
        for (int i=0;i<local_cells;i++)
        {
            for (int j=0;j<local_cells;j++)
                local_map[i][j]=0.0;
        }
        return 0;
    }
    if (worker)
        worker->mutex.unlock();
    return -1;
}
