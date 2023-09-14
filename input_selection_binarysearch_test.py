import csv
import sys
import time
import math
from pymavlink import mavutil
import threading
import numpy as np
import dtw
import pandas as pd
import random
import os
import subprocess
import psutil

# 출력 파일명 설정
output_file_1_1 = 'attitude_log_1_estimated.csv'
output_file_1_2 = 'attitude_log_1_desired.csv'

output_file_2_1 = 'attitude_log_2_estimated.csv'
output_file_2_2 = 'attitude_log_2_desired.csv'

output_file_3_1 = 'attitude_log_3_estimated.csv'
output_file_3_2 = 'attitude_log_3_desired.csv'

output_file_4_1 = 'attitude_log_4_estimated.csv'
output_file_4_2 = 'attitude_log_4_desired.csv'

# 버그 리포트 파일 
bug_report_file = 'bug_report.csv'

# 로그 파일 생성
'''
with open(output_file_1_1, 'w') as f:
    f.write('Timestamp,q1,q2,q3,q4\n')
with open(output_file_1_2, 'w') as f:
    f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')
with open(output_file_2_1, 'w') as f:
    f.write('Timestamp,q1,q2,q3,q4\n')
with open(output_file_2_2, 'w') as f:
    f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')
with open(output_file_3_1, 'w') as f:
    f.write('Timestamp,q1,q2,q3,q4\n')
with open(output_file_3_2, 'w') as f:
    f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')
with open(output_file_4_1, 'w') as f:
    f.write('Timestamp,q1,q2,q3,q4\n')
with open(output_file_4_2, 'w') as f:
    f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')
    '''

# 버그 리포트 파일 생성
with open(bug_report_file, 'w') as f:
    f. write('Case Number,input1,input2\n')


# 드론에 연결
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
all_mission_items_reached = False


total_mission_items = 0
rollrate_target = 0
rollrate_current = 0
rollrate_error = 0
range_max = 1800
range_min = 0
mid_value = random.randint(0,1800)
parameter_incremental = 5

#input_max = 2
input_max = (mid_value + range_max)/2
#input_min = 1
input_min = (mid_value + range_min)/2
prev_input_max = input_max
prev_input_min = input_min

drone_status = 0
system_reboot = 0
crash_failsafe = 0
battery_healthy = 0
running_time_over = 0
logging_started = False
battery_check = 0

#Class for formating the mission item
class mission_item:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.autocontinue = 1
        self.param1 = 0.0 #MAV_CMD_NAV_WAYPOINT hold
        self.param2 = 0.30 #MAV_CMD_NAV_WAYPOINT Accept radius
        self.param3 = 0.50 #MAV_CMD_NAV_WAYPOINT pass radius
        self.param4 = math.nan #MAV_CMD_NAV_WAYPOINT yaw
        self.param5 = x #MAV_CMD_NAV_WAYPOINT latitude
        self.param6 = y #MAV_CMD_NAV_WAYPOINT longitude
        self.param7 = z #MAV_CMD_NAV_WAYPOINT altitude
        self.mission_type = 0 

#Arm the Drone
def arm(the_connection):
    print("--Arming")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)

    ack_with_timeout(the_connection, "COMMAND_ACK")

#Takeoff the Drone
def takeoff(the_connection):
    print("--Takeoff Initiated")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, math.nan, 0, 0, 10)

    ack_with_timeout(the_connection, "COMMAND_ACK")

#Upload the mission items to the drone
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    ack_with_timeout(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:        #Mission Item created base on the Mavlink Message protocol
        print("--Creating a waypoint")

        the_connection.mav.mission_item_send(the_connection.target_system,      #Target system
                                            the_connection.target_component,    #Target component
                                            waypoint.seq,                       
                                            waypoint.frame,
                                            waypoint.command,
                                            waypoint.current,
                                            waypoint.autocontinue,
                                            waypoint.param1,                    #Hold Time
                                            waypoint.param2,                    
                                            waypoint.param3,
                                            waypoint.param4,
                                            waypoint.param5,                    #Local X
                                            waypoint.param6,                    #Local Y
                                            waypoint.param7,                    #Local Z
                                            waypoint.mission_type)

    if waypoint != mission_items[n-1]:
        ack_with_timeout(the_connection, "MISSION_REQUEST")

    ack_with_timeout(the_connection, "MISSION_ACK")


#Send message for the drone to return to the launch point
def set_return(the_connection):
    print("--Set Return To launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0) #MAV_CMD_NAV_RETURN_TO_LAUNCH parameter has 7 values

    ack_with_timeout(the_connection, "COMMAND_ACK")

#Start mission
def start_mission(the_connection):
    print("--Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0) # Why 8 components?

    ack_with_timeout(the_connection, "COMMAND_ACK")

#Acknowledgement from the Drone
def ack_with_timeout(the_connection, keyword, timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        ack_msg = the_connection.recv_match(type=keyword)
        if ack_msg:
            print(f"--ACK received: {ack_msg}")
            break
        time.sleep(0.1)
    else:
        print(f"--ACK timeout for {keyword}")
    #print ("--Mesager Read " + str(the_connection.recv_match(type=keyword, blocking = True)))


def disarm(the_connection):
    print("--Disarming")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)

    ack_with_timeout(the_connection, "COMMAND_ACK")

def land(the_connection):
    print("--Landing")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, math.nan, 0, 0, 0)
    ack_with_timeout(the_connection, "COMMAND_ACK")


#Attitude log record
def log_attitude_1(the_connection, output_file):
    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over
    global logging_started

    msg = None
    #logging_started = False
    print("log1 start")
    with open(output_file_1_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_max, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    start_time = time.time()
    heartbeat_check_time = time.time()
    battery_status_time = time.time()
    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            current_time = time.time()
            run_time = current_time - start_time
            if run_time > 360:
                print("Run time over 5 minutes!")
                running_time_over = 1
                re_launch()
                reboot()
                input_mutation(5)
                break

            if logging_started:
                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1, q2, q3, q4 = msg.q1, msg.q2, msg.q3, msg.q4

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1},{q2},{q3},{q4}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break
            
            heartbeat_check_1 = time.time()

            if msg.get_type() == 'HEARTBEAT':
                drone_status = msg.system_status
                drone_mode_flag = msg.base_mode

                heartbeat_check_time = time.time()

                if drone_status == (6):
                    print("Drone is unsafe!")
                    input_mutation(5)
                    re_launch()
                    reboot()
                    system_reboot = 1
                    crash_failsafe = 1
                    break

                elif drone_mode_flag == (25 or 153):
                    print("Drone Battery unhealthy!")
                    re_launch()
                    reboot()
                    battery_healthy = 1
                    break

            if msg.get_type() == "BATTERY_STATUS":
                battery_mode = msg.mode
                battery_fail = msg.fault_bitmask

                battery_status_time = time.time()

            if abs(heartbeat_check_1 - battery_status_time) > 60:
                print("------Battery Unhealthy!------")
                re_launch()
                reboot()
                battery_healthy = 1
                break
            
            if heartbeat_check_1 - heartbeat_check_time > 20:
                print("Drone heartbeat lost!")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

#Attitude log record
def log_attitude_1_target(the_connection, output_file):
    global system_reboot
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log1 start")
    with open(output_file_1_2, 'w') as f:
        f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if logging_started:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1_d, q2_d, q3_d, q4_d = msg.q

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1_d},{q2_d},{q3_d},{q4_d}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break
            
            if system_reboot == 1:
                break 

            if battery_healthy == 1:
                break

            if running_time_over == 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")



#Attitude log record
def log_attitude_2(the_connection, output_file):
    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log2 start")
    with open(output_file_2_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')
    
    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_min, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    start_time = time.time()
    heartbeat_check_time = time.time()
    battery_status_time = time.time()
    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            current_time = time.time()
            run_time = current_time - start_time
            if run_time > 360:
                print("Run time over 5 minutes!")
                running_time_over = 1
                re_launch()
                reboot()
                input_mutation(6)
                break

            if logging_started:
                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1, q2, q3, q4 = msg.q1, msg.q2, msg.q3, msg.q4

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1},{q2},{q3},{q4}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            heartbeat_check_1 = time.time()

            if msg.get_type() == 'HEARTBEAT':
                drone_status = msg.system_status
                drone_mode_flag = msg.base_mode

                heartbeat_check_time = time.time()

                if drone_status == (6):
                    print("Drone is unsafe!")
                    input_mutation(6)
                    re_launch()
                    reboot()
                    system_reboot = 1
                    crash_failsafe = 1
                    break

                elif drone_mode_flag == (25 or 153):
                    print("Drone Battery unhealthy!")
                    re_launch()
                    reboot()
                    battery_healthy = 1
                    break

            if msg.get_type() == "BATTERY_STATUS":
                battery_mode = msg.mode
                battery_fail = msg.fault_bitmask

                battery_status_time = time.time()

            if abs(heartbeat_check_1 - battery_status_time) > 60:
                print("------Battery Unhealthy!------")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            if heartbeat_check_1 - heartbeat_check_time > 20:
                print("Drone heartbeat lost!")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")


#Attitude log record
def log_attitude_2_target(the_connection, output_file):
    global system_reboot
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log2 start")
    with open(output_file_2_2, 'w') as f:
        f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')

    
    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if logging_started:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1_d, q2_d, q3_d, q4_d = msg.q

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1_d},{q2_d},{q3_d},{q4_d}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            if system_reboot == 1:
                break 

            if battery_healthy == 1:
                break

            if running_time_over == 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

#Attitude log record
def log_attitude_3(the_connection, output_file):
    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log3 start")
    with open(output_file_3_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_max, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    start_time = time.time()
    heartbeat_check_time = time.time()
    battery_status_time = time.time()
    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            current_time = time.time()
            run_time = current_time - start_time
            if run_time > 360:
                print("Run time over 5 minutes!")
                with open(bug_report_file, 'a') as f:
                    f.write(f'{3},{input_max},{input_min}\n')
                running_time_over = 1
                re_launch()
                reboot()
                input_mutation(1)
                break

            if logging_started:
                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1, q2, q3, q4 = msg.q1, msg.q2, msg.q3, msg.q4

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1},{q2},{q3},{q4}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            heartbeat_check_1 = time.time()

            if msg.get_type() == 'HEARTBEAT':
                sys_status = msg.system_status
                drone_mode_flag = msg.base_mode

                heartbeat_check_time = time.time()

                if sys_status == (6):
                    print("Drone is unsafe!")
                    input_mutation(1)
                    with open(bug_report_file, 'a') as f:
                        f.write(f'{3},{input_max},{input_min}\n')
                    re_launch()
                    reboot()
                    system_reboot = 1
                    crash_failsafe = 1
                    break

                elif drone_mode_flag == (25 or 153):
                    print("Drone Battery unhealthy!")
                    with open(bug_report_file, 'a') as f:
                        f.write(f'{3},{input_max},{input_min}\n')
                    re_launch()
                    reboot()
                    battery_healthy = 1
                    break

            if msg.get_type() == "BATTERY_STATUS":
                battery_mode = msg.mode
                battery_fail = msg.fault_bitmask

                battery_status_time = time.time()

            if abs(heartbeat_check_1 - battery_status_time) > 60:
                print("------Battery Unhealthy!------")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            if heartbeat_check_1 - heartbeat_check_time > 20:
                print("Drone heartbeat lost!")
                re_launch()
                reboot()
                battery_healthy = 1
                break


            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")


#Attitude log record
def log_attitude_3_target(the_connection, output_file):
    global system_reboot
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log3 start")
    with open(output_file_3_2, 'w') as f:
        f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if logging_started:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1_d, q2_d, q3_d, q4_d = msg.q

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1_d},{q2_d},{q3_d},{q4_d}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            if system_reboot == 1:
                break 

            if battery_healthy == 1:
                break

            if running_time_over == 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

#Attitude log record
def log_attitude_4(the_connection, output_file):
    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log4 start")
    with open(output_file_4_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_min, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    start_time = time.time()
    heartbeat_check_time = time.time()
    battery_status_time = time.time()
    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            current_time = time.time()
            run_time = current_time - start_time
            if run_time > 360:
                print("Run time over 5 minutes!")
                with open(bug_report_file, 'a') as f:
                    f.write(f'{4},{input_max},{input_min}\n')
                running_time_over = 1
                re_launch()
                reboot()
                input_mutation(1)
                break

            if logging_started:
                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1, q2, q3, q4 = msg.q1, msg.q2, msg.q3, msg.q4

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1},{q2},{q3},{q4}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            heartbeat_check_1 = time.time()

            if msg.get_type() == 'HEARTBEAT':
                sys_status = msg.system_status
                drone_mode_flag = msg.base_mode

                heartbeat_check_time = time.time()

                if sys_status == (6):
                    print("Drone is unsafe!")
                    input_mutation(1)
                    with open(bug_report_file, 'a') as f:
                        f.write(f'{4},{input_max},{input_min}\n')
                    re_launch()
                    reboot()
                    system_reboot = 1
                    crash_failsafe = 1
                    break

                elif drone_mode_flag == (25 or 153):
                    print("Drone Battery unhealthy!")
                    with open(bug_report_file, 'a') as f:
                        f.write(f'{4},{input_max},{input_min}\n')
                    re_launch()
                    reboot()
                    battery_healthy = 1
                    break

            if msg.get_type() == "BATTERY_STATUS":
                battery_mode = msg.mode
                battery_fail = msg.fault_bitmask

                battery_status_time = time.time()

            if abs(heartbeat_check_1 - battery_status_time) > 60:
                print("------Battery Unhealthy!------")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            if heartbeat_check_1 - heartbeat_check_time > 20:
                print("Drone heartbeat lost!")
                re_launch()
                reboot()
                battery_healthy = 1
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

#Attitude log record
def log_attitude_4_target(the_connection, output_file):
    global system_reboot
    global battery_healthy
    global running_time_over
    global logging_started
    
    msg = None
    print("log4 start")
    with open(output_file_4_2, 'w') as f:
        f.write('Timestamp,q1_d,q2_d,q3_d,q4_d\n')

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if logging_started:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    q1_d, q2_d, q3_d, q4_d = msg.q

                    # 로그 기록
                    with open(output_file, 'a') as f:
                        f.write(f'{timestamp},{q1_d},{q2_d},{q3_d},{q4_d}\n')

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 0:
                    logging_started = True

                if seq == (total_mission_items -1):
                    logging_started = False
                    break

            if system_reboot == 1:
                break 

            if battery_healthy == 1:
                break

            if running_time_over == 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

# simulation to check input validation
def case_sim_12(the_connection, mission_waypoints, file1, file2, i):
    
    upload_mission(the_connection, mission_waypoints)
    arm(the_connection)
    takeoff(the_connection)
    start_mission(the_connection)
    
    # 별도의 스레드로 ATTITUDE 메시지 로깅 시작
    log_function_name = f"log_attitude_{i}"
    log_function_name_target = f"log_attitude_{i}_target"

    log_thread = threading.Thread(target=globals()[log_function_name], args=(the_connection, file1))
    log_thread_target = threading.Thread(target=globals()[log_function_name_target], args=(the_connection, file2))
    log_thread.start()
    log_thread_target.start()
    
    # ATTITUDE 메시지 로깅 스레드 종료
    log_thread.join()
    log_thread_target.join()

    print("Logging completed.")
    set_return(the_connection)
    time.sleep(50)

    #착륙
    land(the_connection)
    time.sleep(20)
    disarm(the_connection)
    time.sleep(10)

# simulation to check vulnerabilities
def case_sim_34(the_connection, mission_waypoints, file1, file2, i):
    
    upload_mission(the_connection, mission_waypoints)
    arm(the_connection)
    takeoff(the_connection)
    start_mission(the_connection)
    
    # 별도의 스레드로 ATTITUDE 메시지 로깅 시작
    log_function_name = f"log_attitude_{i}"
    log_function_name_target = f"log_attitude_{i}_target"

    log_thread = threading.Thread(target=globals()[log_function_name], args=(the_connection, file1))
    log_thread_target = threading.Thread(target=globals()[log_function_name_target], args=(the_connection, file2))

    if i == 3:
        monitoring_input_thread = threading.Thread(target=input_update1_2, args=(the_connection,))
    elif i == 4:
        monitoring_input_thread = threading.Thread(target=input_update2_1, args=(the_connection,))
    
    log_thread.start()
    log_thread_target.start()
    monitoring_input_thread.start()
    
    # ATTITUDE 메시지 로깅 스레드 종료
    log_thread.join()
    log_thread_target.join()
    monitoring_input_thread.join()

    print("Logging completed.")
    set_return(the_connection)
    time.sleep(50)

    #착륙
    land(the_connection)
    time.sleep(20)
    disarm(the_connection)
    time.sleep(10)



# Input update 하는 함수
def input_update1_2(the_connection):
    print("input update start")
    msg = None
    rollrate_current = 0
    rollrate_target = 0
    monitoring_start = False

    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over
    

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if monitoring_start:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    rollrate_target = msg.body_roll_rate

                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    timestamp = msg.time_boot_ms
                    rollrate_current = msg.rollspeed

                rollrate_error = rollrate_current - rollrate_target

                if (rollrate_error >= 0.22 or rollrate_error <= -0.22):
                    param_name = b"MC_ROLLRATE_MAX"
                    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_min, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    print(rollrate_error, "parameter update 1 -> 2")

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 1:
                    monitoring_start = True

                if seq == (total_mission_items -1):
                    monitoring_start = False
                    break

            if drone_status == (5 or 6):
                break

            if (battery_healthy or system_reboot or running_time_over or crash_failsafe)== 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")


# Input update 하는 함수
def input_update2_1(the_connection):
    print("input update start")
    msg = None
    monitoring_start = False
    rollrate_current = 0
    rollrate_target = 0

    global drone_status
    global system_reboot
    global crash_failsafe
    global battery_healthy
    global running_time_over

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
            if monitoring_start:
                if msg.get_type() == 'ATTITUDE_TARGET':
                    # mission_item이 업데이트된 경우에만 로그 기록
                    timestamp = msg.time_boot_ms
                    rollrate_target = msg.body_roll_rate

                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    timestamp = msg.time_boot_ms
                    rollrate_current = msg.rollspeed

                rollrate_error = rollrate_current - rollrate_target

                if (rollrate_error >= 0.22 or rollrate_error <= -0.22):
                    param_name = b"MC_ROLLRATE_MAX"
                    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_max, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    print(rollrate_error, "parameter update 2 -> 1")

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 1:
                    monitoring_start = True

                if seq == (total_mission_items -1):
                    break

            if drone_status == (5 or 6):
                break

            if (battery_healthy or system_reboot or running_time_over or crash_failsafe)== 1:
                break

            msg = None

        except KeyboardInterrupt:
            # Ctrl+C로 프로그램 종료
            break

        except Exception as e:
            # 예외 처리
            print(f"Error: {e}")

#DTW is a cost function to compare the two log files.
def dtw_compute():

    vulnerable = False

    # Load the desired and estimated attitude logs for the first case
    desired1 = pd.read_csv(output_file_1_2)
    estimated1 = pd.read_csv(output_file_1_1)

    # Load the desired and estimated attitude logs for the second case
    desired2 = pd.read_csv(output_file_2_2)
    estimated2 = pd.read_csv(output_file_2_1)

    # Load the desired and estimated attitude logs for the third case
    desired3 = pd.read_csv(output_file_3_2)
    estimated3 = pd.read_csv(output_file_3_1)

    # Load the desired and estimated attitude logs for the third case
    desired4 = pd.read_csv(output_file_4_2)
    estimated4 = pd.read_csv(output_file_4_1)


    # Extract the desired and estimated quaternion values for each case
    q1_1d = desired1.iloc[1:-1, 1].astype(float) * 100
    q2_1d = desired1.iloc[1:-1, 2].astype(float) * 100
    q3_1d = desired1.iloc[1:-1, 3].astype(float) * 100
    q4_1d = desired1.iloc[1:-1, 4].astype(float) * 100

    q1_1 = estimated1.iloc[1:-1, 1].astype(float) * 100
    q2_1 = estimated1.iloc[1:-1, 2].astype(float) * 100
    q3_1 = estimated1.iloc[1:-1, 3].astype(float) * 100
    q4_1 = estimated1.iloc[1:-1, 4].astype(float) * 100

    # Extract the desired and estimated quaternion values for the second case
    q1_2d = desired2.iloc[1:-1, 1].astype(float) * 100
    q2_2d = desired2.iloc[1:-1, 2].astype(float) * 100
    q3_2d = desired2.iloc[1:-1, 3].astype(float) * 100
    q4_2d = desired2.iloc[1:-1, 4].astype(float) * 100

    q1_2 = estimated2.iloc[1:-1, 1].astype(float) * 100
    q2_2 = estimated2.iloc[1:-1, 2].astype(float) * 100
    q3_2 = estimated2.iloc[1:-1, 3].astype(float) * 100
    q4_2 = estimated2.iloc[1:-1, 4].astype(float) * 100

    # Extract the desired and estimated quaternion values for the third case
    q1_3d = desired3.iloc[1:-1, 1].astype(float) * 100
    q2_3d = desired3.iloc[1:-1, 2].astype(float) * 100
    q3_3d = desired3.iloc[1:-1, 3].astype(float) * 100
    q4_3d = desired3.iloc[1:-1, 4].astype(float) * 100

    q1_3 = estimated3.iloc[1:-1, 1].astype(float) * 100
    q2_3 = estimated3.iloc[1:-1, 2].astype(float) * 100
    q3_3 = estimated3.iloc[1:-1, 3].astype(float) * 100
    q4_3 = estimated3.iloc[1:-1, 4].astype(float) * 100

    # Extract the desired and estimated quaternion values for the third case
    q1_4d = desired4.iloc[1:-1, 1].astype(float) * 100
    q2_4d = desired4.iloc[1:-1, 2].astype(float) * 100
    q3_4d = desired4.iloc[1:-1, 3].astype(float) * 100
    q4_4d = desired4.iloc[1:-1, 4].astype(float) * 100

    q1_4 = estimated4.iloc[1:-1, 1].astype(float) * 100
    q2_4 = estimated4.iloc[1:-1, 2].astype(float) * 100
    q3_4 = estimated4.iloc[1:-1, 3].astype(float) * 100
    q4_4 = estimated4.iloc[1:-1, 4].astype(float) * 100

    variables = [q1_1d, q2_1d, q3_1d, q4_1d, q1_1, q2_1, q3_1, q4_1, q1_2d, q2_2d, q3_2d, q4_2d, q1_2, q2_2, q3_2, q4_2, q1_3d, q2_3d, q3_3d, q4_3d, q1_3, q2_3, q3_3, q4_3, q1_4d, q2_4d, q3_4d, q4_4d, q1_4, q2_4, q3_4, q4_4]

    # 변수들의 사이즈 확인
    for var in variables:
        if len(var) == 0:
            print("------------Log data was not collected------------")
            return 4


    #q1에 대한 reference, estimated 비교
    q1_dtw1_1 = dtw.dtw(q1_1d, q1_1, keep_internals=True).distance
    q1_dtw1_2 = dtw.dtw(q1_2d, q1_2, keep_internals=True).distance
    q1_dtw1_3 = dtw.dtw(q1_3d, q1_3, keep_internals=True).distance
    q1_dtw1_4 = dtw.dtw(q1_4d, q1_4, keep_internals=True).distance

    #q2에 대한 reference, estimated 비교
    q2_dtw1_1 = dtw.dtw(q2_1d, q2_1, keep_internals=True).distance
    q2_dtw1_2 = dtw.dtw(q2_2d, q2_2, keep_internals=True).distance
    q2_dtw1_3 = dtw.dtw(q2_3d, q2_3, keep_internals=True).distance
    q2_dtw1_4 = dtw.dtw(q2_4d, q2_4, keep_internals=True).distance

    #q3에 대한 reference, estimated 비교
    q3_dtw1_1 = dtw.dtw(q3_1d, q3_1, keep_internals=True).distance
    q3_dtw1_2 = dtw.dtw(q3_2d, q3_2, keep_internals=True).distance
    q3_dtw1_3 = dtw.dtw(q3_3d, q3_3, keep_internals=True).distance
    q3_dtw1_4 = dtw.dtw(q3_4d, q3_4, keep_internals=True).distance

    #q4에 대한 reference, estimated 비교
    q4_dtw1_1 = dtw.dtw(q4_1d, q4_1, keep_internals=True).distance
    q4_dtw1_2 = dtw.dtw(q4_2d, q4_2, keep_internals=True).distance
    q4_dtw1_3 = dtw.dtw(q4_3d, q4_3, keep_internals=True).distance
    q4_dtw1_4 = dtw.dtw(q4_4d, q4_4, keep_internals=True).distance


    print("q1_1 DTW distance value to check validation: ", q1_dtw1_1)
    print("q2_1 DTW distance value to check validation: ", q2_dtw1_1)
    print("q3_1 DTW distance value to check validation: ", q3_dtw1_1)
    print("q4_1 DTW distance value to check validation: ", q4_dtw1_1)
    print("----------------------------")
    print("q1_2 DTW distance value to check validation: ", q1_dtw1_2)
    print("q2_2 DTW distance value to check validation: ", q2_dtw1_2)
    print("q3_2 DTW distance value to check validation: ", q3_dtw1_2)
    print("q4_2 DTW distance value to check validation: ", q4_dtw1_2)
    print("----------------------------")
    print("q1_3 DTW distance value to check validation: ", q1_dtw1_3)
    print("q2_3 DTW distance value to check validation: ", q2_dtw1_3)
    print("q3_3 DTW distance value to check validation: ", q3_dtw1_3)
    print("q4_3 DTW distance value to check validation: ", q4_dtw1_3)
    print("----------------------------")
    print("q1_4 DTW distance value to check validation: ", q1_dtw1_4)
    print("q2_4 DTW distance value to check validation: ", q2_dtw1_4)
    print("q3_4 DTW distance value to check validation: ", q3_dtw1_4)
    print("q4_4 DTW distance value to check validation: ", q4_dtw1_4)
    print("----------------------------")

    # 평가 후 input 변경
    if (q1_dtw1_1 > 1500) or (q2_dtw1_1 > 1500) or (q3_dtw1_1 > 1500) or (q4_dtw1_1 > 1500):
        print("case 1: invalid input was selected")
        # input 1은 유효하지 않음
        global input_max
        #input_max = prev_input_max
        input_max = (mid_value+input_max)/2
        del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
        return 2

    if (q1_dtw1_2 > 1500) or (q2_dtw1_2 > 1500) or (q3_dtw1_2 > 1500) or (q4_dtw1_2 > 1500):
        print("case 2: invalid input was selected")
        # input 2는 유효하지 않음
        global input_min
        #input_min = prev_input_min
        input_min = (mid_value+input_min)/2
        del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
        return 3

    if (q1_dtw1_3 > 1500) or (q2_dtw1_3 > 1500) or (q3_dtw1_3 > 1500) or (q4_dtw1_3 > 1500):
        print("case 3: vulnerable case found!")
        # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함
        input_max
        input_min
        with open(bug_report_file, 'a') as f:
            f.write(f'{3},{input_max},{input_min}\n')
        

    if (q1_dtw1_4 > 1500) or (q2_dtw1_4 > 1500) or (q3_dtw1_4 > 1500) or (q4_dtw1_4 > 1500):
        print("case 4: vulnerable case found!")
        # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함
        input_max
        input_min
        with open(bug_report_file, 'a') as f:
            f.write(f'{4},{input_max},{input_min}\n')
    
    del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
    return 1

def input_mutation(case_num):
    global input_max
    global input_min

    if case_num == 1:
        random_num = random.choice([1, 2])
        if random_num == 1:
            if input_max != range_max:
                input_max = (input_max + range_max)/2
        else:
            if input_min != range_min:
                input_min = (input_min + range_min)/2
    
    elif case_num == 5:
        input_max = (mid_value + input_max)/2
    elif case_num == 6:
        input_min = (mid_value + input_min)/2
    """
    elif case_num == 2:
        input_min = (input_min + mid_value)/2

    
    elif case_num == 3:
        input_max = (input_max + mid_value)/2
        """
    
def bug_report_check(bug_file):
    #만약 똑같은 내용의 bug가 기록되었으면 종료
    rows = []

    with open(bug_file, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row in rows:
                return False
            rows.append(row)

    return True

def re_launch():
    global home_position
    print("----------------------------------------------------------")
    print("--------------------Relaunch the Drone--------------------")
    print("----------------------------------------------------------")
    #sys.exit()

#def msg_handler(msg):

def reboot():
    global system_reboot

    print("----------------------------------------------------------")
    print("--------------------Reboot the system---------------------")
    print("----------------------------------------------------------")
    directory_path = '/home/cps/PX4-Autopilot'
    original_path = '/home/cps/Qgroundcontrol/evaluation'
    os.chdir(directory_path)

    #subprocess.run(["sudo", "pkill", "px4"])
    #subprocess.run(["cps135!"])
    px4_process_name = "px4"

    # PX4 프로세스를 찾아서 종료
    for proc in psutil.process_iter(['pid', 'name']):
        try:
            if "px4" in proc.info['name']:
                print(f"Terminating PX4 process with PID {proc.info['pid']}")
                psutil.Process(proc.info['pid']).terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


    # 실행할 명령어 문자열 생성
    commands = [
        'export PX4_HOME_LAT=35.706537',
        'export PX4_HOME_LON=128.455759',
        'make px4_sitl gazebo'
    ]

    # 명령어 문자열을 세미콜론으로 연결
    full_command = ';'.join(commands)

    # 지정된 디렉토리로 이동 후 명령 실행
    #os.system(f'cd {directory_path} && {full_command}')
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'cd {directory_path} && {full_command}'])

    #subprocess.run(["export", "PX4_HOME_LAT=35.706537"], shell=True, check=True)
    #subprocess.run(["export", "PX4_HOME_LON=128.455759"], shell=True, check=True)
    #subprocess.run(["make", "px4_sitl", "gazebo"], shell=True, check=True)
    print("----------------------------------------------------------")
    print("-------------------------Restart--------------------------")
    print("----------------------------------------------------------")
    system_reboot = 0
    os.chdir(original_path)

#Main Function
if __name__ == "__main__":

    count_loop = 0

    while True:
        #reboot()
        drone_status = 0
        print("--Program Started")
        the_connection = mavutil.mavlink_connection("udp:localhost:14540")
        #the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        #the_connection.mav.NAV_CONTROLLER_OUTPUT.subscribe(the_con악nection.target_system, the_connection.target_component)


        while(the_connection.target_system == 0):
            print("--Checking Heartbeat")
            the_connection.wait_heartbeat() #wait_heartbeat() Finding problem...
            print("--heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

        mission_waypoints = []
        #mission_waypoints = [(35.706537, 128.455759, 10),
        #                    (35.706193, 128.457398, 10),
        #                    (35.706696, 128.456280, 5)]

        # Home position 설정
        #home_position = mission_waypoints[0]
        home_lat = 35.706537
        home_lon = 128.455759
        home_alt = 10
        
        mission_waypoints.append(mission_item(0, 0, home_lat, home_lon, home_alt)) #waypoint,0,Lattitude,longitutde,altitude
        mission_waypoints.append(mission_item(1, 0, 35.706193, 128.457398, 10))
        mission_waypoints.append(mission_item(2, 0, 35.706696, 128.456820, 5))
        
        
        total_mission_items = len(mission_waypoints)  # 미션 아이템의 총 개수

        flag = bug_report_check(bug_report_file)

        

        while flag:

            count_loop += 1 # 반복 횟수 확인
            print(count_loop, " times loop executed")

            # mission 수행
            for i in range (1,3):
                output_file1 = f"output_file_{i}_1"
                output_file2 = f"output_file_{i}_2"
                case_sim_12(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)
                if battery_healthy == 1:
                    #battery_healthy = 0
                    #case_sim_12(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)
                    break
                         
            if battery_healthy == 1:
                battery_healthy = 0
                break
            
            if crash_failsafe == 1:
                crash_failsafe = 0
                break

            if running_time_over == 1:
                running_time_over = 0
                break

            for i in range (3,5):
                output_file1 = f"output_file_{i}_1"
                output_file2 = f"output_file_{i}_2"
                case_sim_34(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)
                if battery_healthy == 1:
                    #battery_healthy = 0
                    #case_sim_12(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)
                    break

            if battery_healthy == 1:
                battery_healthy = 0
                break

            if drone_status == 6:
                re_launch()
                reboot()
                break

            if crash_failsafe == 1:
                crash_failsafe = 0
                break

            if running_time_over == 1:
                running_time_over = 0
                break

            # DTW 계산
            sim_result = dtw_compute()

            if sim_result == 4:
                print("Simulation start again")
                
            else:
                # vulnerable case 여부 파악
                input_mutation(sim_result)

                flag = bug_report_check(bug_report_file)

            #if (input_max == 1800 or input_min == 0):
                #break

        if flag == False:
            print("Same bug reported!")
            print("done!")

        if count_loop > 50:
            print("count_loop: ", count_loop)
            print("done!")
            break
