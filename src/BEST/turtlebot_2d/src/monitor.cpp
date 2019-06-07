#include "monitor.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "monitor");


    monitorAndReport monitorAndReportObject;


    ros::spin();
    return 0;
}
