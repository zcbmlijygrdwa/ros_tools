#ifndef LASWRITTER
#define LASWRITTER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <liblas/liblas.hpp>

#include <string>


void write(pcl::PointCloud<pcl::PointXYZI> const & laserCloudIn, std::string file_name)
{
 try
    {
        char const* name = file_name.c_str();

        std::ofstream ofs;
        if (!liblas::Create(ofs, name))
        {
            throw std::runtime_error(std::string("Can not create ") + name);
        }

        double scale = 100;
        liblas::Header hdr;
        hdr.SetVersionMajor(1);
        hdr.SetVersionMinor(1);
        hdr.SetDataFormatId(liblas::ePointFormat1);
        hdr.SetScale(1.0/scale,1.0/scale,1.0/scale);
        hdr.SetPointRecordsCount(1000); // should be corrected automatically by writer
        liblas::Writer writer(ofs, hdr);

        for(int i = 0 ; i < laserCloudIn.size() ; i++)
        {
            pcl::PointXYZI pcl_p = laserCloudIn.points[i];
            liblas::Point p(&hdr);

            //p.SetCoordinates(10, 20, 30);
            //printv(pcl_p.x);
            //printv(pcl_p.y);
            //printv(pcl_p.z);
            //printvP(pcl_p.intensity);
            //p.SetCoordinates(pcl_p.x*scale, pcl_p.y*scale, pcl_p.z*scale);
            p.SetCoordinates(pcl_p.x, pcl_p.y, pcl_p.z);
            p.SetIntensity ((uint16_t)(pcl_p.intensity*255));

            writer.WritePoint(p);
        }
    }
    catch (std::exception const& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error\n";
    }

}

#endif
