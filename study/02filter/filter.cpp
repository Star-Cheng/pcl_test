// https://www.52txr.cn/2024/keypoints.html
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid体素滤波器
#include <pcl/filters/passthrough.h>                 // PassThrough滤波器
#include <pcl/filters/approximate_voxel_grid.h>      // ApproximateVoxelGrid体素滤波器
#include <pcl/filters/uniform_sampling.h>            // UniformSampling均匀采样滤波器
#include <pcl/filters/statistical_outlier_removal.h> // StatisticalOutlierRemoval统计滤波器
#include <pcl/filters/conditional_removal.h>         // conditional_removal条件滤波器
#include <pcl/filters/radius_outlier_removal.h>      // radius_outlier_removal半径滤波器
#include <pcl/filters/project_inliers.h>             // ProjectInliers投影滤波器
#include <pcl/filters/model_outlier_removal.h>       // ModelOutlierRemoval模型滤波器
#include <pcl/filters/bilateral.h>                   // Bilateral双边滤波器
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr approximate_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr conditional_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projection_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;

    // 1. 直通滤波
    /*直通滤波器的工作原理类似于一个空间的“切片”操作。你可以指定一个轴（例如X轴、Y轴或Z轴）和该轴上的
    一个最小值与最大值，滤波器将仅保留位于这个范围内的点，而移除所有其他的点。例如，如果你在Z轴上应用直
    通滤波器，并设置最小值为0.5米，最大值为1.5米，那么所有Z值不在0.5米到1.5米范围内的点都将被移除*/
    pcl::PassThrough<pcl::PointXYZ> pass; // 实例化一个直通滤波对象
    pass.setInputCloud(cloud);            // 设置输入点云
    pass.setFilterFieldName("z");         // 设置要进行直通滤波的字段为Z轴
    pass.setFilterLimits(0.0, 1.0);       // 设置Z轴的过滤范围为0.0到1.0

    // pass.setFilterLimitsNegative (true); // 如果启用，则会保留指定范围之外的点
    pass.filter(*pass_filtered); // 执行滤波操作，过滤后的点云存储在cloud_filtered

    // 2. 体素滤波
    // VoxelGrid体素滤波器
    pcl::VoxelGrid<pcl::PointXYZ> VoxGrid;
    // 设置输入点云为cloud
    VoxGrid.setInputCloud(cloud);
    // 设置每个体素的大小为0.01米（在X，Y，Z方向）。这个大小决定了滤波的粒度。
    VoxGrid.setLeafSize(0.05f, 0.05f, 0.1f);
    // 执行滤波操作，结果存储在cloud_filtered中
    VoxGrid.filter(*voxel_filtered);

    // Approximate体素格滤波器
    // 创建一个ApproximateVoxelGrid对象approximate_voxel_filter，这个滤波器用于处理pcl::PointXYZ类型的点云
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // 设置每个体素的大小为0.2米（在X，Y，Z方向）
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // 设置输入点云为input_cloud
    approximate_voxel_filter.setInputCloud(cloud);
    // 执行滤波操作，结果存储在fapproximate_voxel_filtered中
    approximate_voxel_filter.filter(*approximate_voxel_filtered);
    // 创建了一个UniformSampling对象，它是模板化的，这里用的模板参数是pcl::PointXYZ，意味着它将作用于包含XYZ坐标的点云
    pcl::UniformSampling<pcl::PointXYZ> filter;

    // 3. 均值滤波
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(0.02); // 采样间隔为0.02
    uniform_sampling.filter(*uniform_filtered);

    // 4. 统计滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
    Statistical.setInputCloud(cloud);
    Statistical.setMeanK(50);            // 设置在计算平均距离时考虑的最近邻点数
    Statistical.setStddevMulThresh(1.0); // 设置判定是否为离群点的标准差倍数
    Statistical.filter(*statistical_filtered);
    // 然后，使用同样的参数再次执行滤波器，但是利用setNegative函数
    // 使得输出点云为原始点云中被认为是离群点的点
    Statistical.setNegative(true);
    Statistical.filter(*statistical_filtered);

    // 5. 条件滤波
    // 创建条件下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    // 添加滤波条件：x轴值大于0.0
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.0)));
    // 添加滤波条件：y轴值小于0.5
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.0)));
    // 添加滤波条件：z轴值小于0.5
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.0)));
    // 创建条件滤波对象并设置条件
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);    // 设置输入点云
    condrem.setKeepOrganized(false); // 如果不需要保持点云结构，设置为false
    // 应用滤波器并保存过滤后的点云到conditional_filtered
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    condrem.filter(*conditional_filtered);

    // 6. 半径滤波
    // 创建一个半径离群值移除滤波器对象
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // 设置输入点云
    outrem.setInputCloud(cloud);
    // 设置在0.8米半径范围内进行搜索，这意味着滤波器会考虑到每个点0.8米范围内的邻近点
    outrem.setRadiusSearch(0.005);
    // 设置一个点至少需要2个邻居点，才能被认为是有效的，不被滤除
    outrem.setMinNeighborsInRadius(3);
    // 应用滤波器并保存过滤后的点云
    outrem.filter(*radius_filtered);

    // 7. 投影滤波
    // 设置ModelCoefficients对象，使用ax+by+cz+d=0平面模型，其中a=b=d=0, c=1，也就是垂直于X-Y平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    // 创建ProjectInliers对象实例，使用ModelCoefficients作为投影对象的模型系数
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);  // 设置投影模型类型为平面
    proj.setInputCloud(cloud);               // 设置输入点云
    proj.setModelCoefficients(coefficients); // 设置模型系数
    proj.filter(*projection_filtered);       // 执行投影滤波，投影结果存储在cloud_projected中

    // 8. 模型滤波
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);                         // 设置球体模型系数大小
    sphere_coeff.values[0] = 0;                            // 球心x坐标
    sphere_coeff.values[1] = 0;                            // 球心y坐标
    sphere_coeff.values[2] = 0;                            // 球心z坐标
    sphere_coeff.values[3] = 0.01;                         // 球体半径
    pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter; // 创建模型离群值移除滤波器
    sphere_filter.setModelCoefficients(sphere_coeff);      // 设置模型系数
    sphere_filter.setThreshold(0.05);                      // 设置阈值为0.05，距离模型超过此值的点将被移除
    sphere_filter.setModelType(pcl::SACMODEL_SPHERE);      // 设置模型类型为球体
    sphere_filter.setInputCloud(cloud);                    // 设置输入点云
    sphere_filter.filter(*cloud_sphere_filtered);          // 执行滤波，将结果存储在cloud_sphere_filtered中

    // 可视化滤波后的点云
    // 定义对象
    pcl::visualization::PCLVisualizer viewer;
    int v1(1); // viewport
    viewer.createViewPort(0.0, 0.5, 0.2, 1.0, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.addPointCloud(cloud, "Original", v1);

    int v2(2); // viewport
    viewer.createViewPort(0.2, 0.5, 0.4, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v2);
    viewer.addPointCloud(pass_filtered, "PassThrough", v2);

    int v3(3); // viewport
    viewer.createViewPort(0.4, 0.5, 0.6, 1.0, v3);
    viewer.setBackgroundColor(0, 0, 0, v3);
    viewer.addPointCloud(voxel_filtered, "Voxel_filtered", v3);

    int v4(4); // viewport
    viewer.createViewPort(0.6, 0.5, 0.8, 1.0, v4);
    viewer.setBackgroundColor(0, 0, 0, v4);
    viewer.addPointCloud(uniform_filtered, "Approximate_voxel_filtered", v4);

    int v5(5); // viewport
    viewer.createViewPort(0.8, 0.5, 1, 1.0, v5);
    viewer.setBackgroundColor(0, 0, 0, v5);
    viewer.addPointCloud(statistical_filtered, "statistical_filtered", v5);

    int v6(6); // viewport
    viewer.createViewPort(0.0, 0.0, 0.2, 0.5, v6);
    viewer.setBackgroundColor(0, 0, 0, v6);
    viewer.addPointCloud(conditional_filtered, "conditional_filtered", v6);

    int v7(7); // viewport
    viewer.createViewPort(0.2, 0.0, 0.4, 0.5, v7);
    viewer.setBackgroundColor(0, 0, 0, v7);
    viewer.addPointCloud(radius_filtered, "radius_filtered", v7);

    int v8(8); // viewport
    viewer.createViewPort(0.4, 0.0, 0.6, 0.5, v8);
    viewer.setBackgroundColor(0, 0, 0, v8);
    viewer.addPointCloud(projection_filtered, "projection_filtered", v8);

    int v9(9); // viewport
    viewer.createViewPort(0.6, 0.0, 0.8, 0.5, v9);
    viewer.setBackgroundColor(0, 0, 0, v9);
    viewer.addPointCloud(cloud_sphere_filtered, "cloud_sphere_filtered", v9);

    int v10(10); // viewport
    viewer.createViewPort(0.8, 0.0, 1.0, 0.5, v10);
    viewer.setBackgroundColor(0, 0, 0, v10);
    viewer.addPointCloud(approximate_voxel_filtered, "approximate_voxel_filtered", v10);

    viewer.spin();

    return (0);
}