"""
This script uses the PositionHlCommander class and logging

It defines a PositionHlCommander setpoint, and records position values.

"""
import cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import time
import math
import logging
from threading import Timer
import numpy as np
import matplotlib.pyplot as plt


# URI to the Crazyflie to connect to
# uri = 'radio://0/80/2M/E7E7E7E703'
# ADDRESS = 0xE7E7E7E703
# 510 Hope, 511 John, 512 Nhat
uri = 'radio://0/80/2M/E7E7E7E510'
ADDRESS = 0xE7E7E7E510
# uri = 'radio://0/80/2M/E7E7E7E710'
# ADDRESS = 0xE7E7E7E710

# uri = 'radio://0/80/2M/E7E7E7E713'
# ADDRESS = 0xE7E7E7E713

logging_period_ms = 10

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Position', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stateEstimate.vx', 'float')
        self._lg_stab.add_variable('stateEstimate.vy', 'float')
        self._lg_stab.add_variable('stateEstimate.vz', 'float')
        # self._lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
        # self._lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
        # self._lg_stab.add_variable('stateEstimateZ.z', 'int16_t')
        # self._lg_stab.add_variable('kalman.stateX', 'float')
        # self._lg_stab.add_variable('kalman.stateY', 'float')
        # self._lg_stab.add_variable('kalman.stateZ', 'float')
        # self._lg_stab.add_variable('kalman.varX', 'float')
        # self._lg_stab.add_variable('kalman.varY', 'float')
        # self._lg_stab.add_variable('kalman.varZ', 'float')
        # self._lg_stab.add_variable('mag.x', 'float')
        # self._lg_stab.add_variable('mag.y', 'float')
        # self._lg_stab.add_variable('mag.z', 'float')


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def ttest_pattern(scf):
    # ## doc: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#controller
    log_config = LogConfig(name='Position', period_in_ms=logging_period_ms)
    # log_config.add_variable('stateEstimate.x', 'float')
    # log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    # log_config.add_variable('stateEstimate.vx', 'float')
    # log_config.add_variable('stateEstimate.vy', 'float')
    # log_config.add_variable('stateEstimate.vz', 'float')
    log_config.add_variable('pm.vbat', 'float')
    # log_config.add_variable('pm.chargeCurrent', 'float')
    # log_config.add_variable('pwm.m1_pwm', 'uint32_t')
    log_config.add_variable('motor.m1', 'uint32_t')
    log_config.add_variable('motor.m2', 'uint32_t')
    log_config.add_variable('motor.m3', 'uint32_t')
    log_config.add_variable('motor.m4', 'uint32_t')
    log_config.add_variable('pm.batteryLevel', 'uint8_t')

    # log_config.add_variable('motor.m1req', 'int32_t')
    # log_config.add_variable('motor.m2req', 'int32_t')
    # log_config.add_variable('motor.m3req', 'int32_t')
    # log_config.add_variable('motor.m4req', 'int32_t')
    # ##### note: only able to log 6 parameters
    data_out=[]
    time_stamp_out = []
    hit_height = False
    with SyncLogger(scf, log_config) as logger:
        with PositionHlCommander(scf) as pc:
            #pc.forward(1.0)
            #pc.left(1.0)
            #pc.back(1.0)
            # pc.go_to(0.0, 0.25, 1, velocity=0.2)

            # step adjusted
            # pc.go_to(0.0, 0, 0.3, velocity=0.8)
            # time.sleep(5)
            # pc.go_to(0.0, 0, 1.3, velocity=1.8)
            # time.sleep(12)

            # step Dr. Rogers
            pc.go_to(0.0, 0, 0.3, velocity=0.8)
            # time.sleep(10)
            time.sleep(5)
            pc.go_to(0.0, 0, 1.3, velocity=1.5)
            time.sleep(12)

            # go forward and backward
            # pc.go_to(0.0, 0, 0.3, velocity=0.8)
            # time.sleep(5)
            # pc.go_to(0.0, 0, 1.3, velocity=1.6)
            # time.sleep(5)
            # pc.go_to(1.0, 0, 1.3, velocity=1.0)
            # time.sleep(5)
            # pc.go_to(-1.0, 0, 1.5, velocity=1.0)
            # time.sleep(10)
            # pc.go_to(0.0, 0, 0.3, velocity=0.8)
            # time.sleep(5)
        for log_entry in logger:
            time_stamp = log_entry[0]
            data = log_entry[1]
            data_out.append(data)
            time_stamp_out.append(time_stamp)
            print(data)
            if data['stateEstimate.z'] > 0.5:
                hit_height = True
            if data['stateEstimate.z'] < 0.05 and hit_height:
                break
    return data_out


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 0.0

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        log_data = ttest_pattern(scf)
    battery_volt = []
    battery_level = []
    m1_pwm = []
    m2_pwm = []
    m3_pwm = []
    m4_pwm = []
    m1_pwm_bat = []
    m2_pwm_bat = []
    m3_pwm_bat = []
    m4_pwm_bat = []
    z_height = []
    z_vel = []
    y_height = []
    y_vel = []
    x_height = []
    x_vel = []
    for data in log_data:
        z_height.append(data['stateEstimate.z'])
        # z_vel.append(data['stateEstimate.vz'])
        # y_height.append(data['stateEstimate.y'])
        # y_vel.append(data['stateEstimate.vy'])
        # x_height.append(data['stateEstimate.x'])
        # x_vel.append(data['stateEstimate.vx'])
        battery_volt.append(data['pm.vbat'])
        m1_pwm.append(data['motor.m1'])
        m2_pwm.append(data['motor.m2'])
        m3_pwm.append(data['motor.m3'])
        m4_pwm.append(data['motor.m4'])
        battery_level.append(data['pm.batteryLevel'])
        # m1_pwm_bat.append(data['motor.m1req'])
        # m2_pwm_bat.append(data['motor.m2req'])
        # m3_pwm_bat.append(data['motor.m3req'])
        # m4_pwm_bat.append(data['motor.m4req'])
        # log_config.add_variable('pwm.m1_pwm', 'uint32_t')
    z_height = np.array(z_height)
    # z_vel = np.array(z_vel)
    battery_volt = np.array(battery_volt)
    battery_level = np.array(battery_level)
    m3_pwm = np.array(m3_pwm)
    t = np.arange(z_height.size)*logging_period_ms      # This is a hack -- there has to be a way to get time from the log

    plt.figure()
    plt.plot(t, np.array(z_height)-0.3)
    # plt.plot(t,np.array(z_vel))
    plt.xlabel('time (sec)')
    plt.ylabel('Amplitude')
    plt.legend((r'$z$ height (m)', r'$\dot{z}$ Vertical Velocity (m/s)'), loc='best', fontsize=13)
    plt.show()

    # plt.figure()
    # plt.plot(t, np.array(x_height))
    # plt.plot(t,np.array(x_vel))
    # plt.xlabel('time (sec)')
    # plt.ylabel('Amplitude')
    # plt.legend((r'$x$ distance (m)', r'$\dot{x}$ Velocity (m/s)'), loc='best', fontsize=13)

    # plt.figure()
    # plt.plot(t, np.array(y_height))
    # plt.plot(t,np.array(y_vel))
    # plt.xlabel('time (sec)')
    # plt.ylabel('Amplitude')
    # plt.legend((r'$y$ distance (m)', r'$\dot{y}$ Velocity (m/s)'), loc='best', fontsize=13)

    def moving_average(a, n=4):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n-1:] / n
    # def moving_average2(a, n=8):
    #     ret = np.cumsum(a, dtype=float)
    #     ret[n:] = ret[n:] - ret[:-n]
    #     return ret[n-1:] / n
    nn = 6  # Delay is 1/2 this number * 10ms
    nnn = 4

    pwm_avg = (np.array(m2_pwm)+np.array(m3_pwm))/2
    pwm_avg_all = (np.array(m1_pwm)+np.array(m2_pwm)+ np.array(m4_pwm) + np.array(m3_pwm)) / 4
    pwm_avg_clean = moving_average((np.array(m2_pwm) + np.array(m3_pwm)) / 2, nn)
    pwm_clean = moving_average(pwm_avg_all,nn)
    plt.figure()
    # plt.plot(t, np.array(battery_volt))
    plt.plot(t, np.array(battery_volt)-1.28)
    # plt.plot(t, moving_average2((2.4*2-np.array(m1_pwm)*3.7/2**16),nnn))
    # plt.plot(t[nn - 1:], moving_average(2.4*2-np.array(m1_pwm)*3.7/2**16, nn))
    # plt.plot(t[nn - 1:], moving_average(2.4 * 2 - np.array(m2_pwm) * 3.7 / 2 ** 16, nn), label= 'm2')
    # plt.plot(t[nn - 1:], moving_average(4.8- np.array(m3_pwm) * 3.7 / 2 ** 16, nn), label='m3')
    # plt.plot(t[nn - 1:], 2.4*2 - pwm_avg_clean * 3.7 / 2**16, label='2 pwm avg clean')
    plt.plot(t[nn - 1:], 4.9 - pwm_clean * 3.7 / 2 ** 16, label='4 pwm avg clean')
    # plt.plot(t[nn - 1:], moving_average(2.4 * 2 - np.array(m4_pwm) * 3.7 / 2 ** 16, nn))
    # plt.plot(t[nn - 1:], moving_average2(2.4 * 2 - np.array(m3_pwm) * 3.7 / 2 ** 16, nnn))

    # plt.plot(t[nn-1.], )
    plt.xlabel('time (sec)')
    plt.ylabel('Amplitude')
    plt.legend()
    # plt.legend((r'Battery (V)', r'motor1 voltage (est)', r'motor2 voltage (est)', r'motor3 voltage (est)',r'motor4 voltage (est)'), loc='best', fontsize=13)
    # plt.legend((r'Battery (V)', r'motor2 voltage (est)', r'motor3 voltage (est) MAV'), loc='best', fontsize=13)
    plt.show()

    # nn = 8
    # plt.figure()
    # # plt.plot(t, np.array(m1_pwm))
    #
    # plt.plot(t, pwm_avg, 'r', label= 'avg of 2 pwm')
    # plt.plot(t, pwm_avg_all, 'g', label = 'avg of 4 pwm')
    #
    # # plt.plot(t[nn-1:], pwm_avg_clean)
    # # plt.plot(t[nn - 1:], pwm_clean)
    # # plt.plot(t, np.array(m2_pwm))
    # # plt.plot(t[nn-1:], moving_average(np.array(m3_pwm), nn))
    # # plt.plot(t, np.array(m4_pwm))
    # plt.xlabel('time (sec)')
    # plt.ylabel('Amplitude rpm')
    # # plt.legend(('motor1', 'motor2', 'motor3', 'motor4'), loc='best')
    # plt.legend(loc='best')
    # plt.show()
    #
    # plt.figure()
    # plt.plot(t, np.array(m1_pwm_bat))
    # plt.plot(t, np.array(m2_pwm_bat))
    # plt.plot(t, np.array(m3_pwm_bat))
    # plt.plot(t, np.array(m4_pwm_bat))
    # plt.legend(('motor1', 'motor2', 'motor3', 'motor4'), loc='best')
    # plt.show()
    # plt.plot(t, np.array(battery_volt))
    # plt.plot(t[nn - 1:], moving_average(2.4 * 2 - np.array(m3_pwm) * 3.7 / 2 ** 16, nn))
    # plt.plot(t[nn - 1:], 2.4 * 2 - moving_average(np.array(m3_pwm), nn) * 3.7 / 2 ** 16, '--')
    # # plt.plot(t, np.array(m4_pwm))
    # plt.xlabel('time (sec)')
    # plt.ylabel('Amplitude')

    # plt.legend('pwm3', loc='best')
    # plt.show()
# ####
    # file_name_str = 'QuadID_11_unit_yz_step_flow_deck_v2_may22_20.txt'
    # file_name_str = 'QuadID_11_unit_z_step_flow_deck_v2_June04_20.txt'
    # file_name_str = 'QuadID_10_unit_z_step_lighthouse_31.7gr_June14_20.txt'
    # file_name_str = 'measured_data_May24_Nhat5.txt'\
    # file_name_str = 'measured_data_May30_Nhat.txt'
    # file_name_str = 'measured_data_June4th_Nhat.txt'
    file_name_str = 'measured_data_June4th_Nhat_2nd.txt'
    # np.savetxt(file_name_str, np.column_stack((t, x_height, y_height, z_height, x_vel, y_vel, z_vel, battery_volt)), delimiter='\t')
    # np.savetxt(file_name_str, np.column_stack((t,  y_height, z_height,  y_vel, z_vel, battery_volt, m1_pwm)), delimiter='\t', fmt='%1.3f')
    # np.savetxt(file_name_str, np.column_stack((t, z_height, z_vel, battery_volt, m3_pwm)), delimiter='\t', fmt='%1.3f')
    np.savetxt(file_name_str, np.column_stack((t, z_height, battery_volt, battery_level, m1_pwm, m2_pwm, m3_pwm, m4_pwm)), delimiter='\t', fmt='%1.3f')
    # np.savetxt(file_name_str, np.column_stack((t, z_height, battery_volt, battery_level, m1_pwm_bat, m2_pwm_bat, m3_pwm_bat, m4_pwm_bat)), delimiter='\t', fmt='%1.3f')
    # m1_pwm_bat
    plt.show()
# questions:
# what is Kalman filter?
# plt.figure()
# plt.plot(t, np.array(battery_volt))
# plt.show()


#
# pwm_avg = (np.array(m2_pwm_bat)+np.array(m3_pwm_bat))/2
# pwm_avg_all = (np.array(m1_pwm_bat)+np.array(m2_pwm_bat)+ np.array(m4_pwm_bat) + np.array(m3_pwm_bat)) / 4
# pwm_avg_clean = moving_average((np.array(m2_pwm_bat) + np.array(m3_pwm_bat)) / 2, nn)
# pwm_clean = moving_average(pwm_avg_all,nn)
# plt.figure()
# plt.plot(t, np.array(battery_volt)-1.28)
# plt.show()
# # plt.plot(t, np.array(battery_level))
# # plt.plot(t, np.array(battery_volt)-1.28)
# # plt.plot(t, moving_average2((2.4*2-np.array(m1_pwm)*3.7/2**16),nnn))
# plt.plot(t[nn - 1:], moving_average(2.4*2-np.array(m1_pwm_bat)*3.7/2**16, nn))
# # plt.plot(t[nn - 1:], moving_average(2.4 * 2 - np.array(m2_pwm) * 3.7 / 2 ** 16, nn), label= 'm2')
# # plt.plot(t[nn - 1:], moving_average(4.8- np.array(m3_pwm) * 3.7 / 2 ** 16, nn), label='m3')
# # plt.plot(t[nn - 1:], 2.4*2 - pwm_avg_clean * 3.7 / 2**16, label='2 pwm avg clean')
# plt.plot(t[nn - 1:], 4.9 - pwm_clean * 3.7 / 2 ** 16, label='4 pwm avg clean')
# # plt.plot(t[nn - 1:], moving_average(2.4 * 2 - np.array(m4_pwm) * 3.7 / 2 ** 16, nn))
# # plt.plot(t[nn - 1:], moving_average2(2.4 * 2 - np.array(m3_pwm) * 3.7 / 2 ** 16, nnn))
#
# # plt.plot(t[nn-1.], )
# plt.xlabel('time (sec)')
# plt.ylabel('Amplitude')
# plt.legend()
# # plt.legend((r'Battery (V)', r'motor1 voltage (est)', r'motor2 voltage (est)', r'motor3 voltage (est)',r'motor4 voltage (est)'), loc='best', fontsize=13)
# # plt.legend((r'Battery (V)', r'motor2 voltage (est)', r'motor3 voltage (est) MAV'), loc='best', fontsize=13)
# plt.figure()
# plt.plot(t, battery_level)
# plt.show()
