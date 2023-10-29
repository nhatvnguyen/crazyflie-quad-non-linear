import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from io import StringIO

# mpl.rc('text', usetex = True)
# # mpl.rc('font', **{'family' : "sans-serif"})
# params= {'text.latex.preamble' : [r'\usepackage{amsmath}']}
# plt.rcParams.update(params)
#
# mpl.rcParams['text.latex.unicode'] = True



#          0     1         2        3          4   5        6
# Data is (t, x_height, y_height, z_height, x_vel, y_vel, z_vel)
# s = StringIO(u"QuadID_10_unit_z_step_lighthouse_31.7gr_June14_20.txt")
# s2 = StringIO(u"simulate_nonlinear_unit_z_step_34.8gr.txt")
# measured_step = np.genfromtxt(s, delimiter='\t') # cant do this for int
# measured_step = np.genfromtxt('./QuadID_10_unit_z_step_lighthouse_31.7gr_June14_20.txt', delimiter='\t')
# measured_step = np.genfromtxt('./QuadID_20_unit_z_step_lighhouse_LED_34.8gr_Aug9_21.txt', delimiter='\t')
# simulated_step = np.genfromtxt('./simulate_nonlinear_unit_z_step_31.7gr.txt', delimiter='\t')
# simulated_step = np.genfromtxt('./simulate_nonlinear_unit_z_step_34.8gr.txt', delimiter='\t')
# simulated_step = np.genfromtxt(s2, delimiter='\t')

# Data is (t, z_height, z_vel, battery_volt, m3_pwm)
measured_step3 = np.genfromtxt('./measured_data_Apr24_Nhat3.txt', delimiter='\t')
measured_step2 = np.genfromtxt('./measured_data_May24_Nhat5.txt', delimiter='\t')
measured_step = np.genfromtxt('./measured_data_Apr24_Nhat4.txt', delimiter='\t')
# measured_step4 = np.genfromtxt('./measured_data_May30_Nhat.txt', delimiter='\t')
measured_step4 = np.genfromtxt('./measured_data_June4th_Nhat.txt', delimiter='\t')
measured_step5 = np.genfromtxt('./measured_data_June4th_Nhat_2nd.txt', delimiter='\t')

# measured (t, z_height, battery_volt, m1_pwm, m2_pwm, m3_pwm, m4_pwm)
simulated_step_non_linear = np.genfromtxt('./simulate_nonlinear_unit_z_step.txt', delimiter='\t')
simulated_step_linear = np.genfromtxt('./simulate_linear_unit_z_step.txt', delimiter='\t')
simulated_step_truelinear = np.genfromtxt('./simulate_truelinear_unit_z_step.txt', delimiter='\t')
index_z_height = 1
index_z_vel = 2
index_volt = 3
ttt = simulated_step_linear[:,0]
tt = simulated_step_non_linear[:,0]
t_lin = simulated_step_truelinear[:,0]
t = measured_step[:, 0]
t = t*(1e-3)
# start_t = 13.65
# start_t = 13.2
start_t = 7.29
# start_t2 = 12.23
start_t2 = 5.0

start_t3 = 0
# start_t = 0

# stop_t = start_t + 12
stop_t = start_t + 30
# stop_t2 = start_t2 + 15
stop_t2 = start_t2 + 12

idx = np.where((t > start_t) & (t < stop_t))
idx2 = np.where((t > start_t2) & (t < stop_t2))
idx3 = np.where((t > start_t3) & (t < stop_t2))
time_z_step = 5-0.3
time_z_step_lin = 5-0.4
# time_z_step = 0.

idx_sim = np.where((tt > time_z_step))       # This comes from the simulation file, and allows the quad to stabalize at hover first
idx_sim_lin = np.where((t_lin > time_z_step_lin))
# idx_sim = np.where((tt > time_z_step) & (tt < stop_t))
# idx_sim = np.where((tt))
plt.figure()
# plt.plot(t[idx] - start_t, measured_step3[idx,index_z_height][0]-0.341, label='Measured on Apr 24th 1st') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)
# plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.341, label='Measured on Apr 24th 2nd') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)
# plt.plot(t[idx2] - start_t2, measured_step[idx2,index_z_height][0]-0.316, label='Measured on May 24th') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)
# plt.plot(t[idx] - start_t, measured_step2[idx,index_z_height][0]-0.316, label='Measured on May 24th') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)
plt.plot(t[idx2] - start_t2, measured_step4[idx2,index_z_height][0]-0.316, label='Measured on June4th') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)
plt.plot(t[idx2] - start_t2, measured_step5[idx2,index_z_height][0]-0.316, label='Measured on June4th_2nd') # plt.plot(t[idx] - start_t, measured_step[idx,index_z_height][0]-0.3)

# plt.plot(t[idx] - start_t, measured_step[idx,index_z_vel][0])
# plt.plot(tt[idx_sim] - time_z_step, simulated_step_non_linear[idx_sim, 3][0]-0.3, linestyle='-.')
#
# plt.plot(tt[idx_sim] - time_z_step, simulated_step_non_linear[idx_sim, 3][0], linestyle='-.', label='Simulated Non-Linear w/o battery correction')
#
plt.plot(ttt[idx_sim]- time_z_step, simulated_step_non_linear[idx_sim, 3][0]-0.3, linestyle='-.', label='Simulated Non-Linear w/ bad battery correction')
# plt.plot(tt[idx_sim] - time_z_step, simulated_step_non_linear[idx_sim, 6][0], linestyle='-.')
#
plt.plot(ttt[idx_sim]- time_z_step, simulated_step_linear[idx_sim, 3][0], linestyle='dotted', label = 'Simulated Non-Linear w/o battery')
plt.plot(t_lin[idx_sim_lin]- time_z_step_lin, simulated_step_truelinear[idx_sim_lin, 1][0]-0.3, linestyle='dotted', label = 'Simulated True Linear')

# interpo
# plt.plot(ttt[idx_sim] - time_z_step, simulated_step_linear[idx_sim, 3][0]-0.3, linestyle='dotted')
# plt.plot(ttt[idx_sim] - time_z_step, simulated_step_linear[idx_sim, 6][0], linestyle='dotted')
# plt.plot(t[idx] - start_t, measured_step[idx,5][0]-3.5)  # what is this?
plt.legend(loc='best')
plt.grid()
plt.title('Measured versus Simulated')
plt.xlabel('time (sec)')
plt.ylabel('Amplitude')
plt.show()
# plt.legend((r'$z$ Measured (m)', r'$\dot{z}$ Velocity Measured (m/s)', r'$z$ Simulated NL (m)', r'$\dot{z}$ Velocity Simulated NL (m/s)', r'$z_{lin}$ height (m) linear', r'$\dot{z_{lin}}$ Vertical Velocity (m/s) linear'), loc='best', fontsize=13)

# plt.figure()
# plt.plot(t[idx] - start_t, measured_step[idx,1][0])
# plt.plot(t[idx] - start_t, measured_step[idx,4][0])
# plt.xlabel('time (sec)')
# plt.ylabel('Amplitude')
# plt.legend((r'$x$ distance (m)', r'$\dot{x}$ Velocity (m/s)'), loc='best', fontsize=13)
# batt voltage
# plt.figure()
# plt.plot(t[idx], measured_step[idx,index_volt][0], label='Measured on Apr 24th 1st')
# plt.plot(t[idx], measured_step2[idx,index_volt][0], label='Measured on May 24th 1st')
# plt.plot(t[idx], measured_step3[idx,index_volt][0], label='Measured on Apr 24th 2nd')
plt.plot(t[idx3], measured_step4[idx3,2][0], label='Battery 1')
plt.plot(t[idx3], measured_step5[idx3,2][0], label='Battery 2')

plt.legend()
plt.xlabel('time (sec)')
plt.ylabel('Amplitude')
plt.title("Measured data from battery")
plt.grid()
plt.show()
# Now do the yz step

#          0     1         2        3          4   5        6
# Data is (t, x_height, y_height, z_height, x_vel, y_vel, z_vel)
# measured_step_yz = np.genfromtxt('./QuadID_11_unit_yz_step_flow_deck_v2_may12_20.txt', delimiter='\t')
# simulated_step_yz = np.genfromtxt('./simulate_nonlinear_unit_yz_step.txt', delimiter='\t')
# tt_yz = simulated_step_yz[:,0]
#
# t_yz = measured_step_yz[:, 0]
# start_t = 13.3
# stop_t = 27.0
# idx = np.where((t_yz > start_t) & (t_yz < stop_t))
# time_z_step = 4.6
# idx_sim = np.where((tt_yz > time_z_step))       # This comes from the simulation file, and allows the quad to stabalize at hover first
#
# plt.figure()
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,3][0]-0.5)
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,6][0])
# plt.plot(tt_yz[idx_sim] - time_z_step, simulated_step_yz[idx_sim, 3][0])
# plt.plot(tt_yz[idx_sim] - time_z_step, simulated_step_yz[idx_sim, 6][0])
# plt.xlabel('time (sec)')
# plt.ylabel('yz Amplitude')
# plt.legend((r'$z$ Measured (m)', r'$\dot{z}$ Velocity Measured (m/s)', r'$z$ Simulated (m)', r'$\dot{z}$ Velocity Simulated (m/s)'), loc='best', fontsize=13)
#
# plt.figure()
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,2][0])
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,5][0])
# plt.plot(tt_yz[idx_sim] - time_z_step, simulated_step_yz[idx_sim, 2][0])
# plt.plot(tt_yz[idx_sim] - time_z_step, simulated_step_yz[idx_sim, 5][0])
# plt.xlabel('time (sec)')
# plt.ylabel('yz Amplitude')
# plt.legend((r'$y$ Measured (m)', r'$\dot{y}$ Velocity Measured (m/s)', r'$y$ Simulated (m)', r'$\dot{y}$ Velocity Simulated (m/s)'), loc='best', fontsize=13)

# plt.figure()
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,1][0])
# plt.plot(t_yz[idx] - start_t, measured_step_yz[idx,4][0])
# plt.xlabel('time (sec)')
# plt.ylabel('Amplitude')
# plt.legend((r'$x$ distance (m)', r'$\dot{x}$ Velocity (m/s)'), loc='best', fontsize=13)


plt.show()
