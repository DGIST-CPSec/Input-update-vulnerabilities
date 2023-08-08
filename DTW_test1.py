#import numpy as np
import dtw
import pandas as pd


#file_path1 = '/home/cps/Qgroundcontrol/vulnerable_case_flight_log/param_update_vul_case1-1/param_update_vul_case1-1_estimator_local_position_0.csv'
#file_path2 = '/home/cps/Qgroundcontrol/vulnerable_case_flight_log/param_update_vul_case1_normal/param_update_vul_case1_normal_estimator_local_position_0.csv'

file1_1 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_1_desired.csv'
file1_2 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_1_estimated.csv'

file2_1 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_2_desired.csv'
file2_2 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_2_estimated.csv'

file3_1 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_3_desired.csv'
file3_2 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_3_estimated.csv'

file4_1 = '/home/cps/Qgroundcontrol/evaluation/attitude_log_4_desired.csv'
file4_2 = '/home/cps/Qgroundcontrol/evalutaion/attitude_log_4_estimated.csv'


#df1 = pd.read_csv(file_path1)
#df2 = pd.read_csv(file_path2)

"""
df1_1 = pd.read_csv(file1_1)
df1_2 = pd.read_csv(file1_2)

df2_1 = pd.read_csv(file2_1)
df2_2 = pd.read_csv(file2_2)

df3_1 = pd.read_csv(file3_1)
df3_2 = pd.read_csv(file3_2)


#desired만 받아옴
q1_1d = df1_1.iloc[1:-1, 1]
q1_2d = df2_1.iloc[1:-1, 1]
q1_3d = df3_1.iloc[1:-1, 1]

q2_1d = df1_1.iloc[1:-1, 2]
q2_2d = df2_1.iloc[1:-1, 2]
q2_3d = df3_1.iloc[1:-1, 2]

q3_1d = df1_1.iloc[1:-1, 3]
q3_2d = df2_1.iloc[1:-1, 3]
q3_3d = df3_1.iloc[1:-1, 3]

q4_1d = df1_1.iloc[1:-1, 4]
q4_2d = df2_1.iloc[1:-1, 4]
q4_3d = df3_1.iloc[1:-1, 4]

#estimated만 받아옴
q1_1 = df1_2.iloc[1:-1, 1]
q1_2 = df2_2.iloc[1:-1, 1]
q1_3 = df3_2.iloc[1:-1, 1]

q2_1 = df1_2.iloc[1:-1, 2]
q2_2 = df2_2.iloc[1:-1, 2]
q2_3 = df3_2.iloc[1:-1, 2]

q3_1 = df1_2.iloc[1:-1, 3]
q3_2 = df2_2.iloc[1:-1, 3]
q3_3 = df3_2.iloc[1:-1, 3]

q4_1 = df1_2.iloc[1:-1, 4]
q4_2 = df2_2.iloc[1:-1, 4]
q4_3 = df3_2.iloc[1:-1, 4]
"""

# Load the desired and estimated attitude logs for the first case
desired1 = pd.read_csv(file1_1)
estimated1 = pd.read_csv(file1_2)

# Load the desired and estimated attitude logs for the second case
desired2 = pd.read_csv(file2_1)
estimated2 = pd.read_csv(file2_2)

# Load the desired and estimated attitude logs for the third case
desired3 = pd.read_csv(file3_1)
estimated3 = pd.read_csv(file3_2)

# Load the desired and estimated attitude logs for the third case
desired4 = pd.read_csv(file4_1)
estimated4 = pd.read_csv(file4_2)


# Extract the desired and estimated quaternion values for each case
q1_1d = desired1.iloc[1:-1, 1]
q2_1d = desired1.iloc[1:-1, 2]
q3_1d = desired1.iloc[1:-1, 3]
q4_1d = desired1.iloc[1:-1, 4]

q1_1 = estimated1.iloc[1:-1, 1]
q2_1 = estimated1.iloc[1:-1, 2]
q3_1 = estimated1.iloc[1:-1, 3]
q4_1 = estimated1.iloc[1:-1, 4]

# Extract the desired and estimated quaternion values for the second case
q1_2d = desired2.iloc[1:-1, 1]
q2_2d = desired2.iloc[1:-1, 2]
q3_2d = desired2.iloc[1:-1, 3]
q4_2d = desired2.iloc[1:-1, 4]

q1_2 = estimated2.iloc[1:-1, 1]
q2_2 = estimated2.iloc[1:-1, 2]
q3_2 = estimated2.iloc[1:-1, 3]
q4_2 = estimated2.iloc[1:-1, 4]

# Extract the desired and estimated quaternion values for the third case
q1_3d = desired3.iloc[1:-1, 1]
q2_3d = desired3.iloc[1:-1, 2]
q3_3d = desired3.iloc[1:-1, 3]
q4_3d = desired3.iloc[1:-1, 4]

q1_3 = estimated3.iloc[1:-1, 1]
q2_3 = estimated3.iloc[1:-1, 2]
q3_3 = estimated3.iloc[1:-1, 3]
q4_3 = estimated3.iloc[1:-1, 4]

# Extract the desired and estimated quaternion values for the third case
q1_4d = desired4.iloc[1:-1, 1]
q2_4d = desired4.iloc[1:-1, 2]
q3_4d = desired4.iloc[1:-1, 3]
q4_4d = desired4.iloc[1:-1, 4]

q1_4 = estimated4.iloc[1:-1, 1]
q2_4 = estimated4.iloc[1:-1, 2]
q3_4 = estimated4.iloc[1:-1, 3]
q4_4 = estimated4.iloc[1:-1, 4]

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


if (q1_dtw1_1 > 15) or (q2_dtw1_1 > 15) or (q3_dtw1_1 > 15) or (q4_dtw1_1 > 15):
    print("case 1: invalid input was selected")
    # input 1은 더이상 증가시키면 안됨

if (q1_dtw1_2 > 15) or (q2_dtw1_2 > 15) or (q3_dtw1_2 > 15) or (q4_dtw1_2 > 15):
    print("case 2: invalid input was selected")
    # input 2는 더이상 증가시키면 안됨

if (q1_dtw1_3 > 15) or (q2_dtw1_3 > 15) or (q3_dtw1_3 > 15) or (q4_dtw1_3 > 15):
    print("case 3: vulnerable case found!")
    # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함

if (q1_dtw1_4 > 15) or (q2_dtw1_4 > 15) or (q3_dtw1_4 > 15) or (q4_dtw1_4 > 15):
    print("case 4: vulnerable case found!")
    # input 1, 2 기록하고 케이스 넘버 또한 기록해야 함

"""
def euclidean_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))

def dtw_distance(X, Y):
    m, n = len(X), len(Y)
    # 2차원 배열로 누적 비용 행렬 초기화
    dp = np.zeros((m+1, n+1))

    # 누적 비용 행렬 계산
    for i in range(1, m+1):
        for j in range(1, n+1):
            cost = euclidean_distance(X[i-1], Y[j-1])
            dp[i][j] = cost + min(dp[i-1][j], dp[i][j-1], dp[i-1][j-1])

    # 최종 DTW 거리 반환
    return dp[m][n]

# 두 시계열 데이터
X = np.array([1, 2, 3, 4, 5])
Y = np.array([2, 4, 6, 8])

distance = dtw_distance(X, Y)
print("DTW 거리:", distance)
"""