#pragma once

#include <string>
#include <vector>
#include <map>

//    CUBE DIMENSIONS                                                            
static constexpr double CUBE_SIZE       = 0.050;
static constexpr double PLACE_CLEARANCE = 0.005;
static constexpr double SURFACE_Z       = 0.001;

//    PYRAMID LAYOUT                                                             
static constexpr double PYRAMID_X = 0.012;
static constexpr double PYRAMID_Y = 0.340;
static constexpr double STEP      = CUBE_SIZE + 0.005;

//    TCP OFFSET                                                                 
static constexpr double GRASP_TCP_TO_CUBE_OFFSET = 0.010;

//    HEIGHT HELPERS                                                             
inline double placeCentreZ(int layer)
{
  return SURFACE_Z + PLACE_CLEARANCE + (layer - 1) * CUBE_SIZE + CUBE_SIZE / 2.0;
}

inline double placeTcpZ(int layer)
{
  return placeCentreZ(layer) + GRASP_TCP_TO_CUBE_OFFSET;
}

struct PyramidCube
{
  std::string name;
  double x, y, z;  
  int layer;
};

inline std::vector<PyramidCube> getPyramidLayout()
{
  return {
    { "cube_1", PYRAMID_X + STEP,       PYRAMID_Y, placeCentreZ(1), 1 },
    { "cube_2", PYRAMID_X,              PYRAMID_Y, placeCentreZ(1), 1 },
    { "cube_3", PYRAMID_X - STEP,       PYRAMID_Y, placeCentreZ(1), 1 },
    { "cube_4", PYRAMID_X + STEP / 2.0, PYRAMID_Y, placeCentreZ(2), 2 },
    { "cube_5", PYRAMID_X - STEP / 2.0, PYRAMID_Y, placeCentreZ(2), 2 },
    { "cube_6", PYRAMID_X,              PYRAMID_Y, placeCentreZ(3), 3 },
  };
}

//    DEPENDENCY GRAPH                                                           
inline std::map<std::string, std::vector<std::string>> getBlockedBy()
{
  return {
    { "cube_1", { "cube_4" } },
    { "cube_2", { "cube_4", "cube_5" } },
    { "cube_3", { "cube_5" } },
    { "cube_4", { "cube_6" } },
    { "cube_5", { "cube_6" } },
    { "cube_6", {} },
  };
}

//    STAGING POSITIONS                                                          
struct StagingSlot
{
  double x, y;
  bool occupied;
};

inline std::vector<StagingSlot> getStagingSlots()
{
  return {
    { 0.200, 0.200, false },
    { 0.200, 0.270, false },
    { 0.200, 0.350, false },
  };
}

//    DELIVERY POSITION                                                          
static constexpr double DELIVERY_X = -0.200;
static constexpr double DELIVERY_Y =  0.200;