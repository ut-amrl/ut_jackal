//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gps_translator.cc
\brief   Translate from GPS to map coordinates
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <stdio.h>

#include <cmath>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "math/math_util.h"
#include "ros/ros_helpers.h"
#include "util/helpers.h"
#include "visualization/visualization.h"

using Eigen::Affine2d;
using Eigen::Rotation2Dd;
using Eigen::Vector2d;
using sensor_msgs::NavSatFix;
using std::string;

using namespace math_util;

DEFINE_string(map, "UT_Campus", "Map name to load");
DEFINE_string(gps_topic, "gps/fix", "ROS topic for GPS messages");
DEFINE_string(maps_dir, "maps", "Maps directory");

string GetMapFileFromName(const string& map) {
  return FLAGS_maps_dir + "/" + map + "/" + map + ".gpsmap.txt";
}

struct GPSTranslator {
  bool Load(const std::string& map) {
    const string file = GetMapFileFromName(map);
    ScopedFile fid(file, "r", true);
    if (fid() == nullptr) return false;
    if (fscanf(fid(), "%lf, %lf, %lf", &gps_origin_latitude,
        &gps_origin_longitude, &map_orientation) != 3) {
      return false;
    }
    printf("Map origin: %12.8lf, %12.8lf\n", gps_origin_latitude, gps_origin_longitude);
    return true;
  }

  Vector2d GPSToMetric(const double latitude, const double longitude) {
    const double theta = DegToRad(latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r = sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double dlat = DegToRad(latitude - gps_origin_latitude);
    const double dlong = DegToRad(longitude - gps_origin_longitude);
    const double r1 = r * c;
    const double x = r1 * dlong;
    const double y = r * dlat;
    return Rotation2Dd(map_orientation) * Vector2d(x, y);
  }

  void MetricToGPS(const Vector2d& loc, double* longitude, double* latitude) {
    const double theta = DegToRad(gps_origin_latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r = sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double r1 = r * c;
    const double dlat = loc.y() / r;
    const double dlong = loc.x() / r1;
    *longitude = gps_origin_longitude + dlong;
    *latitude = gps_origin_latitude + dlat;
  }

  double gps_origin_longitude;
  double gps_origin_latitude;
  double map_orientation;

  // Earth geoid parameters from WGS 84 system
  // https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
  // a = Semimajor (Equatorial) axis
  static constexpr double wgs_84_a = 6378137.0;
  // b = Semiminor (Polar) axis
  static constexpr double wgs_84_b = 6356752.314245;
};

GPSTranslator map_;
ros::Publisher localization_pub_;
ros::Publisher viz_pub_;
amrl_msgs::VisualizationMsg viz_msg_;
amrl_msgs::Localization2DMsg localization_msg_;

void GpsCallback(const NavSatFix& msg) {
  const bool verbose = FLAGS_v > 0;
  if (verbose) {
    printf("Status:%d Service:%d Lat,Long:%12.8lf, %12.8lf Alt:%7.2lf",
          msg.status.status, msg.status.service,
          msg.latitude, msg.longitude, msg.altitude);
  }
  if (msg.status.status == msg.status.STATUS_NO_FIX) {
    if (verbose) printf("\n");
    return;
  }
  const Vector2d p = map_.GPSToMetric(msg.latitude, msg.longitude);
  if (verbose) printf(" X,Y: %9.3lf,%9.3lf\n", p.x(), p.y());
  static const uint32_t kColor = 0x0000C0;
  visualization::ClearVisualizationMsg(viz_msg_);
  visualization::DrawLine((p - Vector2d(0.4, 0)).cast<float>(),
                          (p + Vector2d(0.4, 0)).cast<float>(),
                          kColor,
                          viz_msg_);
  visualization::DrawLine((p - Vector2d(0, 0.4)).cast<float>(),
                          (p + Vector2d(0, 0.4)).cast<float>(),
                          kColor,
                          viz_msg_);
  visualization::DrawLine((p + Vector2d(0.5, 0.5)).cast<float>(),
                          (p + Vector2d(0.5, -0.5)).cast<float>(),
                          kColor,
                          viz_msg_);
  visualization::DrawLine((p + Vector2d(-0.5, 0.5)).cast<float>(),
                          (p + Vector2d(-0.5, -0.5)).cast<float>(),
                          kColor,
                          viz_msg_);
  visualization::DrawLine((p + Vector2d(-0.5, -0.5)).cast<float>(),
                          (p + Vector2d(0.5, -0.5)).cast<float>(),
                          kColor,
                          viz_msg_);
  visualization::DrawLine((p + Vector2d(-0.5, 0.5)).cast<float>(),
                          (p + Vector2d(0.5, 0.5)).cast<float>(),
                          kColor,
                          viz_msg_);
  viz_pub_.publish(viz_msg_);
  localization_msg_.map = FLAGS_map;
  localization_msg_.pose.x = p.x();
  localization_msg_.pose.y = p.y();
  localization_msg_.pose.theta = std::nanf("");
  localization_pub_.publish(localization_msg_);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "gps_translator");
  ros::NodeHandle n;
  ros::Subscriber gps_sub = n.subscribe(FLAGS_gps_topic, 1, &GpsCallback);
  ros_helpers::InitRosHeader("map", &localization_msg_.header);
  viz_msg_ = visualization::NewVisualizationMessage("map", "gps_translator");
  localization_pub_ =
      n.advertise<amrl_msgs::Localization2DMsg>("gps_localization", 1);
  viz_pub_ = n.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  CHECK(map_.Load(FLAGS_map));
  ros::spin();
  return 0;
}
