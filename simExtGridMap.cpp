#include "simExtGridMap.h"
#include "stackArray.h"
#include "stackMap.h"
#include "simLib.h"
#include "gridmap/gridmap.h"
#include <iostream>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif

#if defined(__linux) || defined(__APPLE__)
    #include <unistd.h>
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 5 // 2 since version 3.2.1, 3 since V3.3.1, 4 since V3.4.0, 5 since V3.4.1

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

static GridMap *m_gridmap=NULL;

#define PARAM_GRIDMAP_CELL_SIZE 0
#define PARAM_GRIDMAP_MAP_SIZE 1
#define PARAM_GRIDMAP_PMAX 2
#define PARAM_GRIDMAP_PMIN 3
#define PARAM_GRIDMAP_LASER_MAX_RANGE 4
#define PARAM_GRIDMAP_LASER_PREC 5
#define PARAM_GRIDMAP_LASER_SIGMA 6
#define PARAM_GRIDMAP_LASER_PLAMBDA 7
#define PARAM_GRIDMAP_LASER_WEIGHT_HIT 8
#define PARAM_GRIDMAP_LASER_WEIGHT_UNEXPECTED 9

// --------------------------------------------------------------------------------------
// simExtSkeleton_getData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_GRIDMAP_UPDATE_LASER_COMMAND "simGridMap.updateMapLaser" // the name of the new Lua command
#define LUA_GRIDMAP_INIT_COMMAND "simGridMap.init" // the name of the new Lua command
#define LUA_GRIDMAP_RELEASE_COMMAND "simGridMap.release" // the name of the new Lua command
#define LUA_GRIDMAP_GET_GLOBAL_GRID_COMMAND "simGridMap.getMap" // the name of the new Lua command
#define LUA_GRIDMAP_GET_GLOBAL_POINTS_COMMAND "simGridMap.getPoints" // the name of the new Lua command
#define LUA_GRIDMAP_SAVE_COMMAND "simGridMap.saveMap"
#define LUA_GRIDMAP_LOAD_COMMAND "simGridMap.loadMap"

void LUA_GRIDMAP_UPDATE_LASER_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray* laserScan = new CStackArray();
    CStackArray* laserPose = new CStackArray();    
    CStackArray* point = new CStackArray();
    size_t numPoints=0;
    CStackArray inArguments;
    CStackArray outArguments;

    inArguments.buildFromStack(stack);

    if ((inArguments.getSize()==2)&&inArguments.isArray(0)&&inArguments.isArray(1))
    { // we expect 2 arguments: an array with laser points (x1,y1,x2,y2,...), laser pose (x,y,th)
        laserScan=inArguments.getArray(0);
        laserPose=inArguments.getArray(1);
            numPoints=size_t(laserScan->getSize());
            std::vector<Point_t > laser_ranges;
            laser_ranges.clear();
            Point_t p;
            for (size_t i=0;i<numPoints;i++)
            {
                point=laserScan->getArray(i);
                p.x=point->getDouble(0);
                p.y=point->getDouble(1);
                laser_ranges.push_back(p);
            }
            Pose_t pose;
            pose.x=laserPose->getDouble(0);
            pose.y=laserPose->getDouble(1);
            pose.th=laserPose->getDouble(2);
            m_gridmap->UpdateLaser(laser_ranges,pose);
    }
    else
        simSetLastError(LUA_GRIDMAP_UPDATE_LASER_COMMAND,"Not enough arguments or wrong arguments.");
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_INIT_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    bool showMap;
    GridMapParams_t gridmapParams;
    gridmapParams.cell_size=0.1;
    gridmapParams.map_size=50;
    gridmapParams.pmax=0.9;
    gridmapParams.pmin=0.1;
    gridmapParams.laser_max_range=5;
    gridmapParams.laser_prec=0.1;
    gridmapParams.laser_sigma=0.1;
    gridmapParams.laser_plambda=0.8;
    gridmapParams.laser_weight_hit=0.7;
    gridmapParams.laser_weight_unexpected=0.3;

    CStackArray* gridmap_params = new CStackArray();
    int status=-1;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    if ( (inArguments.getSize()==2)&&inArguments.isArray(0)&&inArguments.isBool(1))
    {
        gridmap_params=inArguments.getArray(0);
        showMap=inArguments.getBool(1);

        if (gridmap_params->getSize()>PARAM_GRIDMAP_CELL_SIZE)
            gridmapParams.cell_size=gridmap_params->getDouble(PARAM_GRIDMAP_CELL_SIZE);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_MAP_SIZE)
            gridmapParams.map_size=gridmap_params->getDouble(PARAM_GRIDMAP_MAP_SIZE);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_PMAX)
            gridmapParams.pmax=gridmap_params->getDouble(PARAM_GRIDMAP_PMAX);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_PMIN)
            gridmapParams.pmin=gridmap_params->getDouble(PARAM_GRIDMAP_PMIN);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_MAX_RANGE)
            gridmapParams.laser_max_range=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_MAX_RANGE);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_PREC)
            gridmapParams.laser_prec=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_PREC);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_SIGMA)
            gridmapParams.laser_sigma=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_SIGMA);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_PLAMBDA)
            gridmapParams.laser_plambda=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_PLAMBDA);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_WEIGHT_HIT)
            gridmapParams.laser_weight_hit=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_WEIGHT_HIT);
        if (gridmap_params->getSize()>PARAM_GRIDMAP_LASER_WEIGHT_UNEXPECTED)
            gridmapParams.laser_weight_unexpected=gridmap_params->getDouble(PARAM_GRIDMAP_LASER_WEIGHT_UNEXPECTED);
        m_gridmap = new GridMap(gridmapParams,showMap);
        m_gridmap->Init();
        status=0;
    }
    else
    {
        char str[100];
        sprintf(str,"Not enough arguments or wrong arguments. %d provided",inArguments.getSize());
        simSetLastError(LUA_GRIDMAP_INIT_COMMAND,str);
    }
    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.pushInt(status);
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_RELEASE_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    if ( (inArguments.getSize()==0))
    { // we expect 0 arguments
        if (m_gridmap!=NULL)
        {
            delete m_gridmap;
            m_gridmap=NULL;
        }
    }
    else
        simSetLastError(LUA_GRIDMAP_RELEASE_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_SAVE_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    if ( (inArguments.getSize()==1)&&inArguments.isString(0))
    { // we expect 0 arguments
        std::string str=inArguments.getString(0);
        m_gridmap->saveMap(str);
    }
    else
        simSetLastError(LUA_GRIDMAP_SAVE_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_LOAD_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);
    int ret=-1;

    if ( (inArguments.getSize()==1)&&inArguments.isString(0))
    { // we expect 0 arguments
        std::string str=inArguments.getString(0);
        ret=m_gridmap->loadMap(str);
    }
    else
        simSetLastError(LUA_GRIDMAP_LOAD_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.pushInt(ret);
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_GET_GLOBAL_GRID_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    bool binarized=false;
    double pmap_threshold=0.65;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);
    CStackArray outArguments;

    if ((inArguments.getSize()==1)&&inArguments.isNumber(0))
    {
        pmap_threshold=inArguments.getDouble(0);
        binarized=true;
    }
    CStackArray *map = new CStackArray();
    m_gridmap->worker->mutex.lock();
    for (unsigned int i=0;i<m_gridmap->map->width();i++)
    {
        CStackArray *map_row = new CStackArray();
        for (unsigned int j=0;j<m_gridmap->map->height();j++)
        {
            CImg<double>::iterator it=m_gridmap->map->begin()+m_gridmap->map->height()*j+i;
            if (binarized)
            {
                if (GridMap::logRatio2Prob(*it)>=pmap_threshold)
                    map_row->pushBool(true);
                else
                    map_row->pushBool(false);
            }
            else
                map_row->pushDouble(GridMap::logRatio2Prob(*it));
        }
        map->pushArray(map_row);
    }
    m_gridmap->worker->mutex.unlock();
    outArguments.pushArray(map);
    outArguments.buildOntoStack(stack);
}

void LUA_GRIDMAP_GET_GLOBAL_POINTS_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    double pmap_threshold=0.65;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);
    CStackArray outArguments;

    if ((inArguments.getSize()==1)&&inArguments.isNumber(0))
    {
        pmap_threshold=inArguments.getDouble(0);
    }
    CStackArray *map = new CStackArray();
    for (unsigned int i=0;i<m_gridmap->map->width();i++)
    {
        for (unsigned int j=0;j<m_gridmap->map->height();j++)
        {
            CImg<double>::iterator it=m_gridmap->map->begin()+m_gridmap->map->height()*j+i;
            if (GridMap::logRatio2Prob(*it)>=pmap_threshold)
            {
                Point_t point;
                point=m_gridmap->cell2Point(i,j);
                CStackArray *map_point = new CStackArray();
                map_point->pushDouble(point.x);
                map_point->pushDouble(point.y);
                map->pushArray(map_point);
            }
        }
    }
    outArguments.pushArray(map);
    outArguments.buildOntoStack(stack);
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind CoppelisSim functions:
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#else
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the CoppelisSim library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the CoppelisSim library:
    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        return(0); // Means error, CoppelisSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        std::cout << "Error, could not find all required functions in the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Check the version of CoppelisSim:
    int simVer,simRev;
    simGetIntegerParameter(sim_intparam_program_version,&simVer);
    simGetIntegerParameter(sim_intparam_program_revision,&simRev);
    if( (simVer<30400) || ((simVer==30400)&&(simRev<9)) )
    {
        std::cout << "Sorry, your CoppelisSim copy is somewhat old, CoppelisSim 3.4.0 rev9 or higher is required. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Implicitely include the script lua/simExtPluginSkeleton.lua:
    simRegisterScriptVariable("simGridMap","require('simExtGridMap')",0);

    // Register the new function:
#ifdef GRIDMAP_DEBUG
    std::cout << "Registering GridMap functions" << std::endl;
#endif
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_UPDATE_LASER_COMMAND,"@","GridMapUpdateLaser"),strConCat("",LUA_GRIDMAP_UPDATE_LASER_COMMAND,"(table laserScan, table laserPose)"),LUA_GRIDMAP_UPDATE_LASER_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_INIT_COMMAND,"@","GridMapInit"),strConCat("number status=",LUA_GRIDMAP_INIT_COMMAND,"(table gridMapParams)"),LUA_GRIDMAP_INIT_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_RELEASE_COMMAND,"@","GridMapRelease"),strConCat("",LUA_GRIDMAP_RELEASE_COMMAND,"()"),LUA_GRIDMAP_RELEASE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_GET_GLOBAL_GRID_COMMAND,"@","GridGetMap"),strConCat("table map=",LUA_GRIDMAP_GET_GLOBAL_GRID_COMMAND,"(number pthreshold)"),LUA_GRIDMAP_GET_GLOBAL_GRID_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_GET_GLOBAL_POINTS_COMMAND,"@","GridGetPoints"),strConCat("table points=",LUA_GRIDMAP_GET_GLOBAL_POINTS_COMMAND,"(number pthreshold)"),LUA_GRIDMAP_GET_GLOBAL_POINTS_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_SAVE_COMMAND,"@","GridSaveMap"),strConCat("",LUA_GRIDMAP_SAVE_COMMAND,"(string filename)"),LUA_GRIDMAP_SAVE_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_GRIDMAP_LOAD_COMMAND,"@","GridLoadMap"),strConCat("number status",LUA_GRIDMAP_LOAD_COMMAND,"(string filename)"),LUA_GRIDMAP_LOAD_CALLBACK);
    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppelisSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
    // Here you could handle various clean-up tasks

    unloadSimLibrary(simLib); // release the library
}

// This is the plugin messaging routine (i.e. CoppelisSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from CoppelisSim (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the CoppelisSim user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // CoppelisSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in CoppelisSim. This message is the most convenient way to do so:

        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
        
    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended

    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)(customData))==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}

