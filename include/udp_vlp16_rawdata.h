#ifndef udp_VELODYNE_POINTCLOUD_RAWDATA_H
#define udp_VELODYNE_POINTCLOUD_RAWDATA_H

#include <velodyne_pointcloud/rawdata.h>
#include "udp_pcap_pointcloud.hpp"

#include "/home/zhenyu/cpp_util/cpp_util.hpp"

namespace velodyne_rawdata
{
    inline float SQR(float val) { return val*val; }

    class UdpRawData : public RawData
    {
        uint32_t last_time = 0;
        public:
            UdpRawData()
            {

            }
            ~UdpRawData()
            {
            }

            void unpack_for_udp(const raw_packet_udp_t* raw, UdpPcapPointCloud& pc)
            {
                pc.clear();
                float azimuth;
                float azimuth_diff;
                int raw_azimuth_diff;
                float last_azimuth_diff=0;
                float azimuth_corrected_f;
                int azimuth_corrected;
                float x, y, z;
                float intensity;

                //printv(sizeof(raw_packet_udp));
                //uint32_t timestamp = raw->status;
                //printv(timestamp);

                //std::string str = "";
                //uint32_t t2 = 0;
                //for(int i = 0 ; i < 32 ; i++)
                //{
                //    t2 <<= 1;
                //    if(timestamp & 1 == 1)
                //    {
                //        //str = "1"+str;
                //        str = str + "1";
                //        t2 |= 1;
                //    }
                //    else
                //    {
                //        //str = "0" + str;
                //        str = str + "0";
                //        t2 |= 0;
                //    }

                //    if((i+1)%8==0)
                //        //str = " "+str;
                //        str = str+" ";

                //     timestamp >>= 1;
                //}

                ////reverse order of bytes in t2
                //printv(t2);
                //uint8_t* t2_ptr = (uint8_t*)&t2;
                //uint8_t temp = t2_ptr[3];
                //t2_ptr[3] = t2_ptr[0];
                //t2_ptr[0] = temp;

                //std::cout<<"temp = "<<+temp<<std::endl;

                //temp = t2_ptr[2];
                //t2_ptr[2] = t2_ptr[1];
                //t2_ptr[1] = temp;

                //std::cout<<"temp = "<<+temp<<std::endl;

                //printv(str);
                //printv(t2);

                //printv("\n\n\n");
                //printv((uint32_t)(raw->status));
                uint32_t time_diff = (uint32_t)(raw->status) - last_time;
                //printv(time_diff);
                last_time = (uint32_t)(raw->status);
                //std::cout<<"return_mode = "<<+(raw->return_mode)<<", product_id = "<<+(raw->product_id)<<std::endl;
                //float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

                for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

                    // ignore packets with mangled or otherwise different contents
                    if (UPPER_BANK != raw->blocks[block].header) {
                        // Do not flood the log with messages, only issue at most one
                        // of these warnings per minute.
                        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                << block << " header value is "
                                << raw->blocks[block].header);
                        return;                         // bad packet: skip the rest
                    }

                    // Calculate difference between current and next block's azimuth angle.
                    azimuth = (float)(raw->blocks[block].rotation);
                    if (block < (BLOCKS_PER_PACKET-1)){
                        raw_azimuth_diff = raw->blocks[block+1].rotation - raw->blocks[block].rotation;
                        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
                        // some packets contain an angle overflow where azimuth_diff < 0 
                        if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
                        {
                            ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
                            // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
                            if(last_azimuth_diff > 0){
                                azimuth_diff = last_azimuth_diff;
                            }
                            // otherwise we are not able to use this data
                            // TODO: we might just not use the second 16 firings
                            else{
                                continue;
                            }
                        }
                        last_azimuth_diff = azimuth_diff;
                    }else{
                        azimuth_diff = last_azimuth_diff;
                    }
                    for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
                        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
                            velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];

                            /** Position Calculation */
                            union two_bytes tmp;
                            tmp.bytes[0] = raw->blocks[block].data[k];
                            tmp.bytes[1] = raw->blocks[block].data[k+1];

                            /** correct for the laser rotation as a function of timing during the firings **/
                            azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
                            azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

                            /*condition added to avoid calculating points which are not
                              in the interesting defined area (min_angle < area < max_angle)*/
                            if ((azimuth_corrected >= config_.min_angle
                                        && azimuth_corrected <= config_.max_angle
                                        && config_.min_angle < config_.max_angle)
                                    ||(config_.min_angle > config_.max_angle
                                        && (azimuth_corrected <= config_.max_angle
                                            || azimuth_corrected >= config_.min_angle))){

                                // convert polar coordinates to Euclidean XYZ
                                float distance = tmp.uint * calibration_.distance_resolution_m;
                                distance += corrections.dist_correction;

                                float cos_vert_angle = corrections.cos_vert_correction;
                                float sin_vert_angle = corrections.sin_vert_correction;
                                float cos_rot_correction = corrections.cos_rot_correction;
                                float sin_rot_correction = corrections.sin_rot_correction;

                                // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
                                // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
                                float cos_rot_angle =
                                    cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                                    sin_rot_table_[azimuth_corrected] * sin_rot_correction;
                                float sin_rot_angle =
                                    sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                                    cos_rot_table_[azimuth_corrected] * sin_rot_correction;

                                float horiz_offset = corrections.horiz_offset_correction;
                                float vert_offset = corrections.vert_offset_correction;

                                // Compute the distance in the xy plane (w/o accounting for rotation)
                                /**the new term of 'vert_offset * sin_vert_angle'
                                 * was added to the expression due to the mathemathical
                                 * model we used.
                                 */
                                float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

                                // Calculate temporal X, use absolute value.
                                float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
                                // Calculate temporal Y, use absolute value
                                float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
                                if (xx < 0) xx=-xx;
                                if (yy < 0) yy=-yy;

                                // Get 2points calibration values,Linear interpolation to get distance
                                // correction for X and Y, that means distance correction use
                                // different value at different distance
                                float distance_corr_x = 0;
                                float distance_corr_y = 0;
                                if (corrections.two_pt_correction_available) {
                                    distance_corr_x =
                                        (corrections.dist_correction - corrections.dist_correction_x)
                                        * (xx - 2.4) / (25.04 - 2.4)
                                        + corrections.dist_correction_x;
                                    distance_corr_x -= corrections.dist_correction;
                                    distance_corr_y =
                                        (corrections.dist_correction - corrections.dist_correction_y)
                                        * (yy - 1.93) / (25.04 - 1.93)
                                        + corrections.dist_correction_y;
                                    distance_corr_y -= corrections.dist_correction;
                                }

                                float distance_x = distance + distance_corr_x;
                                /**the new term of 'vert_offset * sin_vert_angle'
                                 * was added to the expression due to the mathemathical
                                 * model we used.
                                 */
                                xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
                                x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

                                float distance_y = distance + distance_corr_y;
                                /**the new term of 'vert_offset * sin_vert_angle'
                                 * was added to the expression due to the mathemathical
                                 * model we used.
                                 */
                                xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
                                y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

                                // Using distance_y is not symmetric, but the velodyne manual
                                // does this.
                                /**the new term of 'vert_offset * cos_vert_angle'
                                 * was added to the expression due to the mathemathical
                                 * model we used.
                                 */
                                z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;


                                /** Use standard ROS coordinate system (right-hand rule) */
                                float x_coord = y;
                                float y_coord = -x;
                                float z_coord = z;

                                /** Intensity Calculation */
                                float min_intensity = corrections.min_intensity;
                                float max_intensity = corrections.max_intensity;

                                intensity = raw->blocks[block].data[k+2];

                                float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
                                float focal_slope = corrections.focal_slope;
                                intensity += focal_slope * (std::abs(focal_offset - 256 *
                                            SQR(1 - static_cast<float>(tmp.uint) / 65535)));
                                intensity = (intensity < min_intensity) ? min_intensity : intensity;
                                intensity = (intensity > max_intensity) ? max_intensity : intensity;

                                //float time = 0;
                                //if (timing_offsets.size())
                                //  time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

                                UdpPcapPoint point;
                                point.x = x_coord;
                                point.y = y_coord;
                                point.z = z_coord;
                                point.intensity = intensity;
                                point.ring = corrections.laser_ring;
                                pc.push_back(point);
                                //pc.timestamp = (*(unsigned int*)(raw->status))/1000000000.0;
                                pc.timestamp = ((uint32_t)(raw->status))/1000000.0;

                                //printv(x_coord);
                                //printv(y_coord);
                                //printv(z_coord);
                                //printv(corrections.laser_ring);
                                //printv(azimuth_corrected);
                                //printv(distance);
                                //printvP(intensity);

                                //data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time);
                            }
                        }
                        //data.newLine();
                    }
                }
            }
};

}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD_RAWDATA_H
