#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdio.h>
#include <ros.hpp>

//Mine
#include "CostVolume/utils/reproject.hpp"
#include "CostVolume/utils/reprojectCloud.hpp"
#include "CostVolume/Cost.h"
#include "CostVolume/CostVolume.hpp"
#include "Optimizer/Optimizer.hpp"
#include "DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.hpp"
// #include "OpenDTAM.hpp"
#include "set_affinity.h"
#include "Track/Track.hpp"

#include "utils/utils.hpp"

using namespace cv;
using namespace cv::cuda;
using namespace std;

int image_callback(rosmsg image_msg) {
    // Try to detect april tags

    // If april tag found: estimate pose

    // 
}

// FSM:
// SEEK: building cost volume
// behavior: traverse, looking for tags, build cost volume
// transition: if more than one tag seen, and then no tag seen, transition to TRACK

// TRACK: cost volume built, tracking camera movement using cost volume
// behavior: traverse, collecting images and associated poses.
// transition: if too far from original keyframe pose, then start new cost volume using cached frames

// 

int main(int argc, char **argv) {
    // Init ros so that things work
    ros::init(argc, argv, "slammer");

    // get roshandle
    ros::NodeHandle n;

    // create subscriber object
    ros::Subscriber_t s = n.subscribe("image_data", 1); // one image stored only

    // Spin to get it running and calling callbacks
    ros::spin();

	return 0;
}
