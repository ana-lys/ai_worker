#include <iostream>

#include <coal/config.hh>
#include <pinocchio/config.hpp>
#include <mujoco/mujoco.h>
#include "proxsuite/config.hpp"

#ifndef MUJOCO_STAMP_VERSION
#define MUJOCO_STAMP_VERSION "unknown"
#endif

int main()
{
  std::cout << "proxsuite version: " << PROXSUITE_VERSION << std::endl;
  std::cout << "coal version: " << COAL_VERSION << std::endl;
  std::cout << "pinocchio version: " << PINOCCHIO_VERSION << std::endl;
  std::cout << "mujoco version (runtime int): " << mj_version() << std::endl;
  std::cout << "mujoco version (installer stamp): " << MUJOCO_STAMP_VERSION << std::endl;
  return 0;
}
