#!/usr/bin/env python3

import rospy
import numpy as np
# メッセージの型等のimport
from collections import deque
from shipcon_pcc.msg import fusion_msgs
from shipcon_pcc.msg import gps_convert
from geometry_msgs.msg import PoseStamped
from shipcon_pcc.msg import ndt_convert_msgs
#from mavros_msgs.msg import State

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub_fused_position = rospy.Publisher('fused_position', fusion_msgs, queue_size=10)
        self.msg_fused_position = fusion_msgs()
        # messageの型を作成
    def make_msg(self,msg_fused_positon):
        print('a')
    def send_msg(self,msg_fused_position):
        #self.make_msg(self.msg_gps_convert)
        self.pub_fused_position.publish(msg_fused_position)

class Subscribers():
    def __init__(self):
        #importしたメッセージファイルを箱に代入
        self._gps_convert = gps_convert()
        #self.pose = PoseStamped()
        self.ndt_convert_tmp = ndt_convert_msgs()
        # Converted states from GPS
        self.gps_convert = rospy.Subscriber('gps_convert', gps_convert, self.convert_cb)   
        # LiDAR AutoWare
        #self.ndt_pose = rospy.Subscriber('ndt_pose', PoseStamped, self.ndt_pose_cb)
        # Converted states from LiDAR
        self.ndt_convert = rospy.Subscriber('ndt_convert', ndt_convert_msgs, self.ndt_convert_cb)
    def convert_cb(self, msg):
        self._gps_convert = msg
        #print(self._gps_convert.X_chosen)
    #def ndt_pose_cb(self, msg):
        #self.pose = msg
    def ndt_convert_cb(self, msg):
        self.ndt_convert_tmp = msg

class complementary_filter():
    def __init__(self):
        #初期値
        self.x_hat_0 = np.array([[20],[20],[40]])
        self.P_0 = np.array([[1000,0,0],[0,1000,0],[0,0,1000]])

    def linear_kalman_filter(self, x, y, C, A, B, R, I, P, Q):    #yが1次元のときのみ成り立つ
        #予測ステップ
        x = A @ x
        P = A @ P @ A.T + B @ Q @ B.T   
        #フィルタリングステップ
        tmp = C @ P @ C.T + R
        K = (P @ C.T)/tmp
        x = x + K * (y - C @ x)          
        P = (I - K @ C) @ P    
        return x, P

    def refrsh_pos(self, sub):     #センサからのデータを常に更新する
        #GNSSから得た位置情報
        x_pos_gnss = sub._gps_convert.X_chosen
        y_pos_gnss = sub._gps_convert.Y_chosen
        #LiDARのndt_matchingより得た位置情報
        #x_pos_ndt = sub.pose.pose.position.x
        #y_pos_ndt = sub.pose.pose.position.y
        #convertされたLiDARのndt_matchingより得た位置情報
        x_pos_ndt = sub.ndt_convert_tmp.x_pose
        y_pos_ndt = sub.ndt_convert_tmp.y_pose
        z_pos_ndt = sub.ndt_convert_tmp.z_pose
        #LiDAR, GNSSの位置情報の配列
        self.pos_LiDAR = np.array([x_pos_ndt, y_pos_ndt])
        self.pos_GNSS  = np.array([x_pos_gnss, y_pos_gnss])

    def variables(self, a, dim_state_val):
        '''
        観測量y:位置情報のyではない
        相補フィルタでは２つのセンサの差分が観測量になる
        '''
        #観測量y
        y = self.pos_LiDAR - self.pos_GNSS
        ##状態モデル x[k+1] = A * x[k] + B * v[k]  , v ~ N(0, Q)##
        #状態遷移行列
        A = np.array([[a, 0, 0],[0, 1, 0],[0, 0, 0]])
        B = np.array([[1-a, 0],[0, 0],[0, 1]])
        ##観測モデル y[k] = C * x[k] Cは今回単位行列##
        #観測行列
        C = np.array([1, 1, -1])
        ##単位行列##
        I = np.identity((dim_state_val))
        #システム誤差分散行列Q,観測誤差分散行列R
        Q = np.array([[60, 0],[0, 60]])
        R = 10e-2
        return A, B, C, I, Q, R, y

def main():
    # nodeの立ち上げ
    rospy.init_node('NODE_complementary_filter', anonymous = True)
    # インスタンスの作成と実行
    pub = Publishsers()
    sub = Subscribers()
    fusion = complementary_filter()
    #rospy.spin()
    rate = rospy.Rate(10)
    msg_fused = pub.msg_fused_position
    #初期値の実行
    x_hat_0_tmp = fusion.x_hat_0
    x_hat_x = x_hat_0_tmp
    x_hat_y = x_hat_0_tmp
    P_0_tmp = fusion.P_0
    P_x = P_0_tmp
    P_y = P_0_tmp
    while not rospy.is_shutdown():
        #refrsh関数の実行
        fusion.refrsh_pos(sub) 
        #variablesの実行
        A, B, C, I, Q, R, y = fusion.variables(0.8, 3)
        #complementary_filterのlinear_kalman_filterを実行
        x_hat_x, P_x = fusion.linear_kalman_filter(x_hat_x, y[0], C, A, B, R, I, P_x, Q)
        x_hat_y, P_y = fusion.linear_kalman_filter(x_hat_y, y[1], C, A, B, R, I, P_y, Q)
        #出力であるx_hatの割り当て
        mu_hat_x = x_hat_x[0][0]
        beta_hat_x = x_hat_x[1][0]
        e2_hat_x = x_hat_x[2][0]
        mu_hat_y = x_hat_y[0][0]       
        beta_hat_y = x_hat_y[1][0]
        e2_hat_y = x_hat_y[2][0]
        LiDAR_hat = fusion.pos_LiDAR - np.array([mu_hat_x, mu_hat_y]) - np.array([beta_hat_x, beta_hat_y])
        GNSS_hat = fusion.pos_GNSS - np.array([e2_hat_x, e2_hat_y])
        #以下fusionした値をpubするための処理
        msg_fused.x_filtered_LiDAR = LiDAR_hat[0]
        msg_fused.y_filtered_LiDAR = LiDAR_hat[1]
        msg_fused.x_filtered_GNSS = GNSS_hat[0]
        msg_fused.y_filtered_GNSS = GNSS_hat[1]
        print(msg_fused.x_filtered_LiDAR, msg_fused.y_filtered_LiDAR)
        print(msg_fused.x_filtered_GNSS, msg_fused.y_filtered_GNSS)
        pub.send_msg(msg_fused)
        #実行周波数を維持する?
        rate.sleep()

if __name__ == '__main__':
   main()
