#ifndef INCLUDES_H
#define INCLUDES_H


#pragma once

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QDialog>
#include <QString>
#include <QDebug>
#include <QFile>
#include <QDir>

#include "method/set_coefficients.h"
#include "method/method_coefficients.h"
#include "method/StemPointDetection.h"
#include "controller.h"

typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;


#endif // INCLUDES_H

