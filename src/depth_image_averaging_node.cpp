/*
 * This file is part of the depth_image_averaging
 * (https://github.com/salihmarangoz/depth_image_averaging).
 * Copyright (c) 2024 Salih Marangoz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>

#include "nodelet/loader.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_image_averaging_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name,
               "depth_image_averaging/depth_image_averaging_nodelet", remap,
               nargv);
  ros::spin();

  return 0;
}