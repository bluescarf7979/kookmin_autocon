#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

import math
class StanleyControllerS(object):
    '''
    Stanley Contoller for following lane 
    '''
    def __init__(self, timer):
        self.speed = 15
        self.timer = timer
        self.target_speed = {
            'long straight' : 50,
            'long straight2' : 15,
            'short straight': 50,
            'curve': 15,
            'sharp curve1': 40,
            'sharp curve2': 40
       
        }
        self.acc = {
        #    'long straight' : 1.0,
        #     'short straight': -0.4,
        #     'long straight2' : 1,
        #     'curve': None
            'long straight' :None,
            'short straight':None,
            'long straight2' : None,
            'curve': None,
           
           
        }
        self.delay = {
        #    'long straight' : 1.0,
        #     'short straight': 2.5,
        #     'long straight2' : 1,
        #     'curve': 0.5,
            'long straight' : 0.0,
            'short straight': 0.0,
            'long straight2' : 4.0,
            'curve': 0.0,
            
            
        }

    def __call__(self, angle, target, mode):
        '''
        returns angle and speed for following target
        '''

        if self.timer() > self.delay[mode]:
            if self.acc[mode] is None:
                self.speed = self.target_speed[mode]
                # print("input : ",self.speed)
            elif self.acc[mode] > 0:
                self.speed = min(self.speed+self.acc[mode], self.target_speed[mode])
            else:
                self.speed = max(self.speed+self.acc[mode], self.target_speed[mode])

        yaw_term = angle
        cte = (target-320) * 1.9/650.0
        stanley_k = 1.3 if (mode == 'curve') else 1 #** stanley K? ???? ????!!
        cte_term = math.atan2(stanley_k*cte, 3.0)
        return math.degrees(0.4 * yaw_term + cte_term) * 5.0 / 3.0 + 2, self.speed



# #! /usr/bin/env python
# # -*- coding:utf-8 -*-

# import numpy as np
# import math

# class StanleyControllerS(object):
#     '''
#     Stanley Contoller for following lane 
#     '''
#     def __init__(self, timer):
#         self.speed = 20
#         self.timer = timer
#         self.target_speed = {
#             'long straight' : 50,
#             'short straight': 45,
#             'curve': 35,
#         }
#         self.acc = {
#            'long straight' : 1.0,
#             'short straight': -0.4,
#             'curve': None,
#         }
#         self.delay = {
#            'long straight' : 1.0,
#             'short straight': 2.5,
#             'curve': -1,
#         }

#     def __call__(self, angle, target, mode):
#         '''
#         returns angle and speed for following target
#         '''

#         if self.timer() > self.delay[mode]:
#             if self.acc[mode] is None:
#                 self.speed = self.target_speed[mode]
#             elif self.acc[mode] > 0:
#                 self.speed = min(self.speed+self.acc[mode], self.target_speed[mode])
#             else:
#                 self.speed = max(self.speed+self.acc[mode], self.target_speed[mode])

#         yaw_term = angle
#         cte = (target-320) * 1.9/650.0
#         stanley_k = 1.0 if (mode == 'curve') else 0.7
#         cte_term = math.atan2(stanley_k*cte, 3.0)
#         return math.degrees(0.4 * yaw_term + cte_term) * 5.0 / 3.0 + 2, self.speed

