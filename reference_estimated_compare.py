import time
import math
from pymavlink import mavutil
import threading
import numpy as np
import dtw
import pandas as pd
import random


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
input_1 = 230
input_2 = 80

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
    msg = None
    logging_started = False
    print("log1 start")
    with open(output_file_1_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
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
    msg = None
    logging_started = False
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
    msg = None
    logging_started = False
    print("log2 start")
    with open(output_file_2_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_2, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
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
    msg = None
    logging_started = False
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
    msg = None
    logging_started = False
    print("log3 start")
    with open(output_file_3_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')

    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
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
    msg = None
    logging_started = False
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
    msg = None
    logging_started = False
    print("log4 start")
    with open(output_file_4_1, 'w') as f:
        f.write('Timestamp,q1,q2,q3,q4\n')


    # 초기 설정하고 미션 수행
    print("initial parameter set")
    param_name = b"MC_ROLLRATE_MAX"
    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_2, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # ATTITUDE 메시지 구독
    while True:
        try:
            # 메시지 수신 (로그 주기 설정에 따라 메시지가 더 자주 수신됨)
            while msg is None:
                msg = the_connection.recv_match()
            
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
    msg = None
    logging_started = False
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
    logging_started = False
    rollrate_current = 0
    rollrate_target = 0

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
                    rollrate_target = msg.body_roll_rate

                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    timestamp = msg.time_boot_ms
                    rollrate_current = msg.rollspeed

                rollrate_error = rollrate_current - rollrate_target

                if (rollrate_error >= 0.18 or rollrate_error <= -0.18):
                    param_name = b"MC_ROLLRATE_MAX"
                    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_2, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    print(rollrate_error, "parameter update 1 -> 2")

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 1:
                    logging_started = True

                if seq == (total_mission_items -1):
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
    logging_started = False
    rollrate_current = 0
    rollrate_target = 0

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
                    rollrate_target = msg.body_roll_rate

                if msg.get_type() == 'ATTITUDE_QUATERNION':
                    timestamp = msg.time_boot_ms
                    rollrate_current = msg.rollspeed

                rollrate_error = rollrate_current - rollrate_target

                if (rollrate_error >= 0.18 or rollrate_error <= -0.18):
                    param_name = b"MC_ROLLRATE_MAX"
                    the_connection.mav.param_set_send(the_connection.target_system, the_connection.target_component, param_name, input_1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    print(rollrate_error, "parameter update 2 -> 1")

            
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                print(seq)

                if seq == 1:
                    logging_started = True

                if seq == (total_mission_items -1):
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
    q1_1d = desired1.iloc[1:-1, 1].astype(float)
    q2_1d = desired1.iloc[1:-1, 2].astype(float)
    q3_1d = desired1.iloc[1:-1, 3].astype(float)
    q4_1d = desired1.iloc[1:-1, 4].astype(float)

    q1_1 = estimated1.iloc[1:-1, 1].astype(float)
    q2_1 = estimated1.iloc[1:-1, 2].astype(float)
    q3_1 = estimated1.iloc[1:-1, 3].astype(float)
    q4_1 = estimated1.iloc[1:-1, 4].astype(float)

    # Extract the desired and estimated quaternion values for the second case
    q1_2d = desired2.iloc[1:-1, 1].astype(float)
    q2_2d = desired2.iloc[1:-1, 2].astype(float)
    q3_2d = desired2.iloc[1:-1, 3].astype(float)
    q4_2d = desired2.iloc[1:-1, 4].astype(float)

    q1_2 = estimated2.iloc[1:-1, 1].astype(float)
    q2_2 = estimated2.iloc[1:-1, 2].astype(float)
    q3_2 = estimated2.iloc[1:-1, 3].astype(float)
    q4_2 = estimated2.iloc[1:-1, 4].astype(float)

    # Extract the desired and estimated quaternion values for the third case
    q1_3d = desired3.iloc[1:-1, 1].astype(float)
    q2_3d = desired3.iloc[1:-1, 2].astype(float)
    q3_3d = desired3.iloc[1:-1, 3].astype(float)
    q4_3d = desired3.iloc[1:-1, 4].astype(float)

    q1_3 = estimated3.iloc[1:-1, 1].astype(float)
    q2_3 = estimated3.iloc[1:-1, 2].astype(float)
    q3_3 = estimated3.iloc[1:-1, 3].astype(float)
    q4_3 = estimated3.iloc[1:-1, 4].astype(float)

    # Extract the desired and estimated quaternion values for the third case
    q1_4d = desired4.iloc[1:-1, 1].astype(float)
    q2_4d = desired4.iloc[1:-1, 2].astype(float)
    q3_4d = desired4.iloc[1:-1, 3].astype(float)
    q4_4d = desired4.iloc[1:-1, 4].astype(float)

    q1_4 = estimated4.iloc[1:-1, 1].astype(float)
    q2_4 = estimated4.iloc[1:-1, 2].astype(float)
    q3_4 = estimated4.iloc[1:-1, 3].astype(float)
    q4_4 = estimated4.iloc[1:-1, 4].astype(float)

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
    if (q1_dtw1_1 > 15) or (q2_dtw1_1 > 15) or (q3_dtw1_1 > 15) or (q4_dtw1_1 > 15):
        print("case 1: invalid input was selected")
        # input 1은 더이상 증가시키면 안됨
        global input_1
        input_1 -= 10
        del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
        return 2

    if (q1_dtw1_2 > 15) or (q2_dtw1_2 > 15) or (q3_dtw1_2 > 15) or (q4_dtw1_2 > 15):
        print("case 2: invalid input was selected")
        # input 2는 더이상 증가시키면 안됨
        global input_2
        input_2 += 10
        del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
        return 3

    if (q1_dtw1_3 > 15) or (q2_dtw1_3 > 15) or (q3_dtw1_3 > 15) or (q4_dtw1_3 > 15):
        print("case 3: vulnerable case found!")
        # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함
        input_1
        input_2
        with open(bug_report_file, 'a') as f:
            f.write(f'{3},{input_1},{input_2}\n')
        

    if (q1_dtw1_4 > 15) or (q2_dtw1_4 > 15) or (q3_dtw1_4 > 15) or (q4_dtw1_4 > 15):
        print("case 4: vulnerable case found!")
        # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함
        input_1
        input_2
        with open(bug_report_file, 'a') as f:
            f.write(f'{4},{input_1},{input_2}\n')
    
    del desired1, desired2, desired3, desired4, estimated1, estimated2, estimated3, estimated4
    return 1

def input_mutation(case_num):
    global input_1
    global input_2

    if case_num == 1:
        random_num = random.choice([1, 2])
        if random_num == 1:
            input_1 += 10
        else:
            input_2 -= 10

    elif case_num == 2:
        input_2 -= 10
    
    elif case_num == 3:
        input_1 += 10
    

#Main Function
if __name__ == "__main__":
    print("--Program Started")
    the_connection = mavutil.mavlink_connection("udp:localhost:14540")
    #the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    #the_connection.mav.NAV_CONTROLLER_OUTPUT.subscribe(the_con악nection.target_system, the_connection.target_component)


    while(the_connection.target_system == 0):
        print("--Checking Heartbeat")
        the_connection.wait_heartbeat() #wait_heartbeat() Finding problem...
        print("--heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    mission_waypoints = []

    # Home position 설정
    home_lat = 35.706537  # 출발지점 위도
    home_lon = 128.455759  # 출발지점 경도
    home_alt = 10  # 출발지점 고도

    mission_waypoints.append(mission_item(0, 0, home_lat, home_lon, home_alt)) #waypoint,0,Lattitude,longitutde,altitude
    mission_waypoints.append(mission_item(1, 0, 35.706193, 128.457398, 10))
    mission_waypoints.append(mission_item(2, 0, 35.706696, 128.456820, 5))
    
    total_mission_items = len(mission_waypoints)  # 미션 아이템의 총 개수

    while True:
        # mission 수행
        for i in range (1,3):
            output_file1 = f"output_file_{i}_1"
            output_file2 = f"output_file_{i}_2"
            case_sim_12(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)
        
        for i in range (3,5):
            output_file1 = f"output_file_{i}_1"
            output_file2 = f"output_file_{i}_2"
            case_sim_34(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)

        # DTW 계산
        sim_result = dtw_compute()

        # vulnerable case 여부 파악
        input_mutation(sim_result)

        if (input_1 == 1800 or input_2 == 0):
            break

    print("done!")
