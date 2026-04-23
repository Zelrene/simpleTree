#include "stem_fit.h"

Stem_fit::Stem_fit(PointCloudI::Ptr treeCloud, float min_height, float bin_width, float epsilon)
{
    this->_bin_width = bin_width;
    this->_min_height = min_height;
    this->_cloud = treeCloud;
    this->_epsilon = epsilon;
    compute();
}

void
Stem_fit::compute()
{
    float lower_height = 0.0f;
    float upper_height = lower_height + _bin_width;
    while((lower_height+upper_height)/2 < _min_height)
    {
        _temp_cloud.reset(new PointCloudI);
        pcl::PassThrough<PointI> pass;
        pass.setInputCloud (_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (lower_height, upper_height);
        pass.filter (*_temp_cloud);
        fit_circle(_temp_cloud, lower_height,upper_height, _epsilon);



        lower_height += _bin_width;
        upper_height += _bin_width;
    }
    _upper_cloud.reset(new PointCloudI);
    pcl::PassThrough<PointI> pass;
    pass.setInputCloud (_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((lower_height+upper_height)/2-_bin_width, 100);
    pass.filter (*_upper_cloud);

    if(circles.size()>1)
    {
        for(size_t i = 0; i < circles.size()-1; i++ )
        {
            pcl::ModelCoefficients circle_1 = circles.at(i);
            float x1 = circle_1.values.at(0);
            float y1 = circle_1.values.at(1);
            float z1 = circle_1.values.at(2);
            float r1 = circle_1.values.at(3);
            pcl::ModelCoefficients circle_2 = circles.at(i+1);
            float x2 = circle_2.values.at(0);
            float y2 = circle_2.values.at(1);
            float z2 = circle_2.values.at(2);
            float r2 = circle_2.values.at(3);
            pcl::ModelCoefficients cylinder;
            cylinder.values.resize(7);
            cylinder.values.at(0) = x1;
            cylinder.values.at(1) = y1;
            cylinder.values.at(2) = z1;
            cylinder.values.at(3) = x2-x1;
            cylinder.values.at(4) = y2-y1;
            cylinder.values.at(5) = z2-z1;
            cylinder.values.at(6) = (r1+r2)/2;
            cylinders.push_back(cylinder);
        }
    }

}

void
Stem_fit::fit_circle(PointCloudI::Ptr cloud, float lower_height, float upper_height, float epsilon)
{
        // Safety Check
        if (!cloud || cloud->empty())
        {
            std::cerr << "[fit_circle] ERROR: empty input cloud.\n";
            return;
        }

        // Remove NaNs and infs (for PCL RANSAC stability)
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);

        if(cloud->empty())
        {
            std::cerr << "[fit_circle] ERROR: cloud empty after NaN removal.\n";
            return;
        }

        // Build 2D Projection (XY only)
        PointCloudI::Ptr cloud_2d(new pcl::PointCloud<PointI>);
        cloud_2d->reserve(cloud->size());

        float z_plane = (lower_height + upper_height) / 2.0;

        for (const auto& p : cloud->points)
        {
            PointI q;
            q.x = p.x;
            q.y = p.y;
            q.z = z_plane;  // flatten to a slice plane
            cloud_2d->push_back(q);
        }

        if (cloud_2d->size() < 1)
        {
            std::cerr << "[fit_circle] ERROR: too few points for circle fit.\n";
            return;
        }

        // OPTIONAL: DOWN-SAMPLING STABILITY CHECK
        pcl::VoxelGrid<PointI> vg;
        vg.setInputCloud(cloud_2d);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);

        PointCloudI::Ptr cloud_filtered(new pcl::PointCloud<PointI>);
        vg.filter(*cloud_filtered);

        if (cloud_filtered->size() < 1)
            cloud_filtered = cloud_2d;

        // RANSAC Circle fit (2D)
        pcl::SACSegmentation<PointI> seg;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);

        seg.setMaxIterations(200);
        seg.setDistanceThreshold(epsilon);

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        // Validation of result
        if (inliers->indices.empty())
        {
            std::cerr << "[fit_circle] WARNING: no circle found\n";
            return;
        }

        if (coefficients->values.size() < 3)
        {
            std::cerr << "[fit_circle] ERROR: invalid coefficients\n";
            return;
        }

        // Store Result
        circles.push_back(*coefficients);

        std::cout << "[fit_circle] success: "
                << "center=(" << coefficients->values[0] << ", "
                << coefficients->values[1] << ") "
                << "radius=" << coefficients->values[2]
                << " inliers=" << inliers->indices.size()
                << std::endl;

}
