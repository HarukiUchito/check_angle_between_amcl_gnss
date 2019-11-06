#include "lla.hpp"
#include "utility.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

int main() 
{
    std::string line;
    std::string filename = "/home/uchiito-h/catkin_ws/src/check_angle_between_amcl_gnss/dump/gnss_lla.txt";
    std::string outname = "/home/uchiito-h/catkin_ws/src/check_angle_between_amcl_gnss/dump/converted_gnss.txt";

    std::ifstream param_file("/home/uchiito-h/catkin_ws/src/check_angle_between_amcl_gnss/param/params.txt");
    getline (param_file, line); std::cout << line << std::endl; double ini_lat = std::stod(line);
    getline (param_file, line); double ini_lon = std::stod(line);
    getline (param_file, line); double ini_alt = std::stod(line);
    getline (param_file, line); double yaw = std::stod(line);

    LLA initial_lla(ini_lat, ini_lon, ini_alt);

    geometry_msgs::TransformStamped tr;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tr.transform.rotation = tf2::toMsg(q);
    
    std::ifstream ifs(filename);
    std::ofstream ofs(outname);

    int cnt = 0;
    while (std::getline(ifs, line))
    {
        std::stringstream ss(line);
        double lat, lon, alt;
        ss >> lat >> lon >> alt;

        LLA current_lla(lat, lon, alt);
        // lla -> enu
        geometry_msgs::Point pos {CalcRelativePosition(initial_lla, current_lla)};
        // enu -> map
        geometry_msgs::Point ret;
        tf2::doTransform(pos, ret, tr);

        //std::cout << "x: " << ret.x << ", y: " << ret.y << std::endl;
        ofs << ret.x << " " << ret.y << std::endl;

        cnt++;
    }

    std::cout << cnt << " lines" << std::endl;

    return 0;
}