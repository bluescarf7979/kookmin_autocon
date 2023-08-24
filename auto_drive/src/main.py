#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from justparrel import Parrel
from ar_tracking import ar_tracking
from verticalmine import Vertical
from jangemool import Obstacle
from cross_walk import Cross_walk_mission 
from cone_ing import cone_mission
'''''

'''''


if __name__ == "__main__" :
    rospy.init_node('Mission_driving_node',anonymous=True)

    print('=================== Mission drive start ======================')

    Parrel()
    ar_tracking()
    Vertical()
    Obstacle()
    Cross_walk_mission()
    cone_mission()



    print("==================== Finish =====================")
