#ifndef UdpPcapPointCloud_HPP
#define UdpPcapPointCloud_HPP

struct UdpPcapPoint
{
    float x = 0;
    float y = 0;
    float z = 0;
    float intensity = 0;
    int ring = 0;
};

class UdpPcapPointCloud
{
    public:
        double timestamp = 0;
        
        std::vector<UdpPcapPoint> points;

        int size()
        {
            return points.size();
        }

        void push_back(UdpPcapPoint& point)
        {
            points.push_back(point);
        }

        void clear()
        {
            points.clear();
        }
};

#endif
