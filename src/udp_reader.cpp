#include <fstream>
#include <iterator>
#include <vector>
#include <iostream>
#include "/home/zhenyu/cpp_util/cpp_util.hpp"

#include <velodyne_pointcloud/rawdata.h>

main(int ac, char** av)
{

    printv(ac);
    if(ac<=1)
    {
        std::cout<<"Please enter file path to pcap file.\n";
        return -1;
    }
    printv(av[1]);

    FILE *fileptr;
    char *buffer;
    long filelen;

    fileptr = fopen(av[1], "rb");  // Open the file in binary mode
    fseek(fileptr, 0, SEEK_END);          // Jump to the end of the file
    filelen = ftell(fileptr);             // Get the current byte offset in the file
    rewind(fileptr);                      // Jump back to the beginning of the file

    static const uint16_t UPPER_BANK = 0xeeff; // two bytes
    static const char EE = 0xee; // 1 byte
    static const char FF = 0xff; // 1 byte
    static const int PACKET_SIZE = 1206;

    buffer = (char *)malloc(filelen * sizeof(char)); // Enough memory for the file
    fread(buffer, filelen, 1, fileptr); // Read in the entire file
    fclose(fileptr); // Close the file

    int x =buffer[0];
    int y =buffer[1];

    uint16_t z = ((uint16_t*)buffer)[0];

    printv(x);
    std::cout << std::hex << x << std::endl;
    std::cout << std::hex << y << std::endl;
    std::cout << std::hex << z << std::endl;

    printv(x==FF);
    printv(y==EE);

    std::cout << std::dec<<std::endl;
    printv(filelen);
    printv(PACKET_SIZE);
    printv(filelen%PACKET_SIZE);
    long num_packet = filelen/PACKET_SIZE;
    printv(num_packet);
    printv(sizeof(velodyne_rawdata::raw_packet_t));
    velodyne_rawdata::raw_packet_t* packet_ptr = nullptr;
    velodyne_rawdata::RawData rd;
    rd.setParameters(0,500,0,0);
    rd.setupOffline("/home/zhenyu/catkin_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml", 500,0);

    double start_time = -1;
    double curr_time = -1;


    //search for first FFEE
    for(int i = 0 ; i < filelen ; i++)
    {
        x=buffer[i];
        y=buffer[i+1];
        if(x==FF && y==EE)
        {
            std::cout<<"Found FFEE at i = "<<i<<std::endl;
            //check if packet starts from here by checking end
            int return_mode = buffer[i+1204];
            int product_id = buffer[i+1205];
            //printv(return_mode);
            //printv(product_id);

            if(return_mode==55 && product_id == 34)
            {
                std::cout<<"Found return mode and product_id match!"<<std::endl;
                printv(return_mode);
                printv(product_id);
                printv(packet_ptr);
                packet_ptr = (velodyne_rawdata::raw_packet_t*)(buffer+i);
                int timestamp = buffer[i+1200];
                int timestamp4b = *((int*)(buffer+(i+1200)));
                //rd.unpack_vlp16_udp(packet_ptr);
                printv(timestamp);
                printv(timestamp4b);
                if(start_time<0)
                {
                    start_time = timestamp4b/1000000.0f;
                }
                curr_time = timestamp4b/1000000.0f;
            }


        }
    }

    printv(start_time);
    printv(curr_time);
    double diff_time = curr_time - start_time;
    printvP(diff_time);


    for(int i = 0 ; i < filelen ; i+=PACKET_SIZE)
    {
        x=buffer[i];
        y=buffer[i+1];
        if(x==FF && y==EE)
        {
            printv(i);

            packet_ptr = (velodyne_rawdata::raw_packet_t*)(buffer+i);

            printv(packet_ptr);

            rd.unpack_vlp16_udp(packet_ptr);


        }
        else
        {
            std::cout<<"Corrupted data detected."<<std::endl;
            break;
        }
    }


    return 0;
}
