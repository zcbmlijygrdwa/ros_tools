#ifndef UdpPcapReader_HPP
#define UdpPcapReader_HPP

#include "udp_pcap_pointcloud.hpp"
#include "udp_vlp16_rawdata.h"

#include "/home/zhenyu/cpp_util/cpp_util.hpp"

class UdpPcapReader
{

    public:

        std::string data_path = "";

        FILE *fileptr;
        char *buffer = nullptr;
        long filelen;
        long num_packet = 0;

        velodyne_rawdata::raw_packet_t* packet_ptr = nullptr;
        velodyne_rawdata::UdpRawData rd;

        int count = 0;

        static const uint16_t UPPER_BANK = 0xeeff; // two bytes
        static const char EE = 0xee; // 1 byte
        static const char FF = 0xff; // 1 byte
        static const int PACKET_SIZE = 1206;

        ~UdpPcapReader()
        {
            if(buffer!=nullptr)
            {
                free(buffer);
            }
        }

        void setDataPath(std::string path)
        {
            data_path = path;
            fileptr = fopen(data_path.c_str(), "rb");  // Open the file in binary mode
            fseek(fileptr, 0, SEEK_END);          // Jump to the end of the file
            filelen = ftell(fileptr);             // Get the current byte offset in the file
            rewind(fileptr);                      // Jump back to the beginning of the file
            buffer = (char *)malloc(filelen * sizeof(char)); // Enough memory for the file
            fread(buffer, filelen, 1, fileptr); // Read in the entire file
            fclose(fileptr); // Close the file

            num_packet = filelen/PACKET_SIZE;

            rd.setParameters(0,500,0,0);
            rd.setupOffline("/home/zhenyu/catkin_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml", 500,0);

        }


        bool empty()
        {
            //printv(count);
            return count>=num_packet;
        }

        UdpPcapPointCloud getPc()
        {
            UdpPcapPointCloud pc;
            for(int i = 0 ; i < 5 &&(!empty()) ; i++)
            {
                packet_ptr = (velodyne_rawdata::raw_packet_t*)(buffer)+count;
                rd.unpack_for_udp(packet_ptr, pc);
                count++;
            }

            return pc;
        }
};

#endif
