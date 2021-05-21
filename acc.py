#!/usr/bin/env python3
# coding=UTF-8
# -*- coding: utf-8 -*-
# version: v0.0.1
# author: lian tao
# contact: liantao@edu.bme.hu
# project: acc
# filename: acc.py
# datetime: 2021-05-09 00:10
# description: this script is for acc in sumo

import os, sys
import time
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci


# print(traci.vehicle.getPosition())
class ACC_control():
    def __init__(self, ego_vehicle, front_vehicle):
        self.ego = ego_vehicle
        self.front = front_vehicle
        self.ego_speed = traci.vehicle.getSpeed(self.ego)
        self.front_speed = traci.vehicle.getSpeed(self.front)
        self.vmax = traci.vehicle.getMaxSpeed(self.ego)
        self.maxacc = traci.vehicle.getAccel(self.ego)
        self.maxdec = traci.vehicle.getDecel(self.ego)
        self.length = traci.vehicle.getLength(self.ego)

    def get_relative_distance(self):
        (x0, y0) = traci.vehicle.getPosition(self.front)
        (x1, y1) = traci.vehicle.getPosition(self.ego)
        g = x0 - x1 - self.length
        # add some noise
        g = g * random.uniform(0.9, 1.1)
        return g

    def next_speed(self):
        min_relative_length = self.ego_speed / (0.447 * 10)
        if min_relative_length < 2:
            min_relative_length = 2
        current_relative_length = self.get_relative_distance()
        if current_relative_length > min_relative_length:
            v_next = min(self.ego_speed + self.maxacc, self.vmax)
        elif current_relative_length < min_relative_length:
            v_next = max(self.ego_speed - self.maxdec, 0)
        else:
            v_next = self.ego_speed
        return v_next


sumoCmd = ["sumo-gui", "-c", "acc.sumocfg", "--start"]
traci.start(sumoCmd)
traci.gui.setSchema("View #0", "real world")
print("Starting SUMO")

i = 0
while i < 100:
    time.sleep(0.5)
    traci.simulationStep()
    vehicles = traci.vehicle.getIDList()
    # print(vehicles)
    front_car = vehicles[0]
    if len(vehicles) == 2:
        ego_car = vehicles[1]
        ACC = ACC_control(ego_car, front_car)
        distance = ACC.get_relative_distance()
        current_front_vehicle = traci.vehicle.getSpeed(front_car)
        current_ego_vehicle = traci.vehicle.getSpeed(ego_car)
        print('current speed:', traci.vehicle.getSpeed(front_car), traci.vehicle.getSpeed(ego_car))
        traci.vehicle.setSpeedMode(ego_car, 0)
        traci.vehicle.setLaneChangeMode(ego_car, 0)
        v_next = ACC.next_speed()
        traci.vehicle.setSpeed(ego_car, v_next)
        next_front_vehicle = random.randint(10, 30)
        traci.vehicle.setSpeed(front_car, next_front_vehicle)
        print('next speed,distance', (ACC.front_speed, v_next, distance))
        # file.write(str(current_ego_vehicle)+','+str(current_front_vehicle)+','+str(v_next)+','+str(distance)+'\n')
    else:
        traci.vehicle.setSpeed(front_car, 20)
    i = i + 1
traci.close()

# This is for test 2
# This is for second revise