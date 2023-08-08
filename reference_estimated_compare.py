import time
import math
from pymavlink import mavutil
import threading
import numpy as np
import dtw
import pandas as pd


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

# 버그 리포트 파일 생성
with open(bug_report_file, 'w') as f:
    f. write('Case Number,input1,input2\n')


# 드론에 연결
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
all_mission_items_reached = False
global total_mission_items
total_mission_items = 0


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

    ack(the_connection, "COMMAND_ACK")

#Takeoff the Drone
def takeoff(the_connection):
    print("--Takeoff Initiated")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, math.nan, 0, 0, 10)

    ack(the_connection, "COMMAND_ACK")

#Upload the mission items to the drone
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    ack(the_connection, "MISSION_REQUEST")

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
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")


#Send message for the drone to return to the launch point
def set_return(the_connection):
    print("--Set Return To launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0) #MAV_CMD_NAV_RETURN_TO_LAUNCH parameter has 7 values

    ack(the_connection, "COMMAND_ACK")

#Start mission
def start_mission(the_connection):
    print("--Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0) # Why 8 components?

    ack(the_connection, "COMMAND_ACK")

#Acknowledgement from the Drone
def ack(the_connection, keyword):
    print ("--Mesager Read " + str(the_connection.recv_match(type=keyword, blocking = True)))

def get_setpoint(the_connection):
    print("--Getting ")

def disarm(the_connection):
    print("--Disarming")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)

    ack(the_connection, "COMMAND_ACK")

def land(the_connection):
    print("--Landing")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, math.nan, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")


#Attitude log record
def log_attitude_1(the_connection, output_file):
    msg = None
    logging_started = False
    print("log1 start")
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

def case_sim(the_connection, mission_waypoints, file1, file2, i):
    
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


#DTW is a cost function to compare the two log files.
def dtw_compute(file1, file2):
    df1 = pd.read_csv(file1)
    df2 = pd.read_csv(file2)

    roll_1 = df1.iloc[1:-1, 1]
    roll_2 = df2.iloc[1:-1, 1]

    pitch_1 = df1.iloc[1:-1, 2]
    pitch_2 = df2.iloc[1:-1, 2]

    yaw_1 = df1.iloc[1:-1, 3]
    yaw_2 = df2.iloc[1:-1, 3]

    roll_dtw = dtw.dtw(roll_1, roll_2, keep_internals=True).distance
    pitch_dtw = dtw.dtw(pitch_1, pitch_2, keep_internals=True).distance
    yaw_dtw = dtw.dtw(yaw_1, yaw_2, keep_internals=True).distance

    print("Roll DTW distance value: ", roll_dtw)
    print("Pitch DTW distance value: ", pitch_dtw)
    print("Yaw DTW distance value: ", yaw_dtw)


#Main Function
if __name__ == "__main__":
    print("--Program Started")
    the_connection = mavutil.mavlink_connection("udp:localhost:14540")
    #the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    #the_connection.mav.NAV_CONTROLLER_OUTPUT.subscribe(the_connection.target_system, the_connection.target_component)


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
    mission_waypoints.append(mission_item(1, 0, 35.706053, 128.457817, 10))
    mission_waypoints.append(mission_item(2, 0, 35.706615, 128.456980, 5))
    
    total_mission_items = len(mission_waypoints)  # 미션 아이템의 총 개수

    for i in range (1,5):
        output_file1 = f"output_file_{i}_1"
        output_file2 = f"output_file_{i}_2"
        case_sim(the_connection, mission_waypoints, globals()[output_file1], globals()[output_file2], i)

    print("done!")
