#!/usr/bin/env python3  

import sys
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
import rclpy

"""
SCRIPT DE TESTE PARA VOOS AUTÔNOMOS

O drone irá decolar com altura de 3 metros e irá 2 metros para frente com velocidade de 1 m/s, 
ficará em hold por 5 segundos e depois pousará

Instruções de como rodar no README da mavbase2 (https://github.com/SkyRats/mavbase2)

"""

def run():
    rclpy.init()
    mav = MAV2()
    mav.get_logger().info("Drone will take off to 3 m and move 2 m forward and then land")
    mav.takeoff(3)
    mav.go_to_local(2, 0, mav.drone_pose.pose.position.z, 0, 1)
    mav.hold(5)
    mav.land()

if __name__ == "__main__":
    run()