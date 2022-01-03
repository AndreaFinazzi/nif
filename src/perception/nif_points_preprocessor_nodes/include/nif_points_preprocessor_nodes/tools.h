#include <tf2_ros/transform_listener.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/filters/approximate_voxel_grid.h>

struct LuminarPointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  // float ring;
  // float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LuminarPointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)); 
                                      // (float, ring, ring)(float, time,
                                                                    // time));

namespace nif {
namespace perception {
namespace tools {


/**
 * @brief Transform list for the three Luminar units of the AV-21 racecar (2021/12/27) 
 * 
 * @return std::vector<tf2::Transform> 
 */
std::vector<tf2::Transform> get_av21_lidar_transform_list() {
    
    std::vector<tf2::Transform> transfrom_list;
  tf2::Transform transform_to_center_of_gravity;
  transform_to_center_of_gravity.setOrigin(
      tf2::Vector3(-1.3206, 0.030188, -0.23598));
  tf2::Quaternion quat_to_center_of_gravity;
  quat_to_center_of_gravity.setRPY(0.0, 0.0, 0.0);
  transform_to_center_of_gravity.setRotation(quat_to_center_of_gravity);

  // Lidar Rotate
  tf2::Transform transform_lidar_rotate;
  transform_lidar_rotate.setOrigin(tf2::Vector3(0, 0, 0));
  tf2::Quaternion quat_lidar_rotate;
  // quat_lidar_rotate.setRPY(0.0, 0.0, -M_PI / 2); //origin data was rotated
  // but our sensory data was okay
  quat_lidar_rotate.setRPY(0.0, 0.0, 0.0);
  transform_lidar_rotate.setRotation(quat_lidar_rotate);

  //  FRONT LIDAR
  tf2::Transform transform_front;
  transform_front.setOrigin(tf2::Vector3(2.242, 0, 0.448));
  tf2::Quaternion quat_front;
  quat_front.setRPY(0.0, 0.0, 0.0);
  transform_front.setRotation(quat_front);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_front *
                           transform_lidar_rotate);

  //  LEFT LIDAR
  tf2::Transform transform_left;
  transform_left.setOrigin(tf2::Vector3(1.549, 0.267, 0.543));
  tf2::Quaternion quat_left;
  quat_left.setRPY(0.0, 0.0, 2.0943951024);
  transform_left.setRotation(quat_left);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_left *
                           transform_lidar_rotate);

  //  RIGHT LIDAR
  tf2::Transform transform_right;
  transform_right.setOrigin(tf2::Vector3(1.549, -0.267, 0.543));
  tf2::Quaternion quat_right;
  quat_right.setRPY(0.0, 0.0, -2.0943951024);
  transform_right.setRotation(quat_right);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_right *
                           transform_lidar_rotate);

    return transfrom_list;
}

void transformLuminarPointCloudCustom(
    const pcl::PointCloud<LuminarPointXYZIRT> &cloud_in,
    pcl::PointCloud<LuminarPointXYZIRT> &cloud_out,
    const tf2::Transform &transform) {
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order,
  // despite the ordering of arguments in Eigen's constructor. We could use an
  // Eigen Map to convert without copy, but this only works if Bullet uses
  // floats, that is if BT_USE_DOUBLE_PRECISION is not defined. Rather that
  // risking a mistake, we copy the quaternion, which is a small cost compared
  // to the conversion of the point cloud anyway. Idem for the origin.
  tf2::Quaternion q = transform.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(),
                              q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  // Eigen::Transform3f t;
  // t = translation * rotation;
  transformPointCloud(cloud_in, cloud_out, origin, rotation);
}

void transformLuminarPointCloudCustom(
    const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
    pcl::PointCloud<pcl::PointXYZI> &cloud_out,
    const tf2::Transform &transform) {
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order,
  // despite the ordering of arguments in Eigen's constructor. We could use an
  // Eigen Map to convert without copy, but this only works if Bullet uses
  // floats, that is if BT_USE_DOUBLE_PRECISION is not defined. Rather that
  // risking a mistake, we copy the quaternion, which is a small cost compared
  // to the conversion of the point cloud anyway. Idem for the origin.
  tf2::Quaternion q = transform.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(),
                              q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  // Eigen::Transform3f t;
  // t = translation * rotation;
  transformPointCloud(cloud_in, cloud_out, origin, rotation);
}

} // namespace tools
} // namespace perception
} //namespace nif