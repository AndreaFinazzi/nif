//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/12/21.
//

#ifndef NIF_FRAME_ID_H
#define NIF_FRAME_ID_H

#include <iostream>
#include <string>

namespace nif
{
    namespace common
    {
        namespace frame_id
        {

            /**
 * Center of the vehicle
 */
            std::string VEHICLE_BASE = "base_link";

            /**
 * Middel of the Rear Axil of the vehicle
 */
            std::string VEHICLE_REAR = "rear_axis_middle";

            /**
 * Middel of the Front Axil of the vehicle
 */
            std::string VEHICLE_FRONT = "front_axis_middle";

            namespace lidar
            {
                /**
 * Front facing Lidar (Luminar Hydra)
 */
                std::string LIDAR_FRONT = "front_lidar";

                /**
 * Left-Rear facing Lidar (Luminar Hydra)
 */
                std::string LIDAR_LEFT_REAR = "left_rear_lidar";

                /**
 * Right-Rear facing Lidar (Luminar Hydra)
 */
                std::string LIDAR_RIGHT_REAR = "right_rear_lidar";
            } // namespace lidar

            namespace camera
            {
                /**
 * Front-Left-Center Camera (Mako-G Gige)
 */
                std::string CAMERA_FRONT_LEFT_CENTER = "front_left_center_camera";

                /**
 * Front-Right-Center Camera (Mako-G Gige)
 */
                std::string CAMERA_FRONT_RIGHT_CENTER = "front_right_center_camera";

                /**
 * Front-Left Camera (Mako-G Gige)
 */
                std::string CAMERA_FRONT_LEFT = "front_left_camera";

                /**
 * Front-Right Camera (Mako-G Gige)
 */
                std::string CAMERA_FRONT_RIGHT = "front_right_camera";

                /**
 * Rear-Left Camera (Mako-G Gige)
 */
                std::string CAMERA_REAR_LEFT = "rear_left_camera";

                /**
 * Rear-right Camera (Mako-G Gige)
 */
                std::string CAMERA_REAR_RIGHT = "rear_right_camera";
            } // namespace camera

            namespace radar
            {
                /**
 * Front radar
 */
                std::string RADAR_FRONT = "front_radar";
                /**
 * Rear radar
 */
                std::string RADAR_REAR = "rear_radar";
                /**
 * Left radar
 */
                std::string RADAR_LEFT = "left_radar";
                /**
 * Right radar
 */
                std::string RADAR_RIGHT = "right_radar";
            } // namespace radar

            namespace gps
            {
                /**
 * TODO: should be renamed more clearly
 * GPS antenas which are installed in left and right
 */
                std::string GPS_HORIZONTAL = "gps_horizontal";

                /**
 * TODO: should be renamed more clearly
 * GPS antenas which are installed in front and rear
 */
                std::string GPS_VERTICAL = "gps_vertical";
            } // namespace gps

            namespace localization
            {
                /**
 * Global coordinates : "odom"
 * Body coordinates : "base_link" , "base_link" is indentical to "center_of_gravity"
 */
                std::string ODOM = "odom";

                /**
 * Body coordinates : "base_link"
 */
                std::string BASE_LINK = "base_link";

                /**
 * Currently, "base_link" is indentical to "center_of_gravity"
 */
                std::string CENTER_OF_GRAVITY = BASE_LINK;
            }

        } // namespace frame_id
    }     // namespace common
} // namespace nif

#endif // NIF_FRAME_ID_H
