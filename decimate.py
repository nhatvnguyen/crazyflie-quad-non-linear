import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# wave_duration = 3
sample_rate = 100
# freq = 2
q = 5


simulated_step_non_linear = np.genfromtxt('./simulate_nonlinear_unit_z_step.txt', delimiter='\t')
simulated_step_linear = np.genfromtxt('./simulate_linear_unit_z_step.txt', delimiter='\t')
simulated_step_truelinear = np.genfromtxt('./simulate_truelinear_unit_z_step_new.txt', delimiter='\t')
measured_step4 = np.genfromtxt('./measured_data_May30_Nhat.txt', delimiter='\t')
# simulated_step_truelinear_dec = signal.decimate(simulated_step_truelinear[:,1],2)

start_t = 0.00
start_t2 = 12.23
# start_t2 = 13
stop_t2 = start_t2 + 13.3
# idx2 = np.where((t > start_t2) & (t < stop_t2))

t = measured_step4[:, 0]*(1e-3) # time for the measured
tt = simulated_step_non_linear[:, 0]
ttt = simulated_step_linear[:, 0]
t_lin = simulated_step_truelinear[:,0]
# t_lin_dec = signal.decimate(t_lin,2)
# t = t




time_z_step = 5-0.3
time_z_step_lin = 5-0.4
time_z_stop = time_z_step+13.14 # stop time for sim
time_z_stop_lin = time_z_step_lin+13.23

# initial index from beginning
idx = np.where((tt > start_t))
idx_meas = np.where((t > start_t2) & (t < stop_t2))
# idx_sim_lin = np.where((t_lin_dec > start_t))

index_z_height = 1
# samples_decimated = int(samples/q)
# non_linear_decimated = signal.decimate(simulated_step_non_linear[idx, 3][0], q)
non_linear_decimated = signal.decimate(simulated_step_non_linear[idx, 3][0], q)
fake_linear_decimated = signal.decimate(simulated_step_linear[idx, 3][0], q)
# true_linear_decimated = signal.decimate(simulated_step_truelinear_dec[idx_sim_lin], q)

tt_new = signal.decimate(tt[idx], q)
# tt_lin = signal.decimate(t_lin[idx_sim_lin], q)
idx_sim = np.where((tt_new > time_z_step) & (tt_new < time_z_stop)) # cut idx
idx_lin = np.where((t_lin > time_z_step_lin) & (t_lin < time_z_stop_lin)) # cut idx 3.962

# tt_new = tt/q
measured_height = measured_step4[idx_meas, index_z_height][0]-0.316
sim_height =  non_linear_decimated[idx_sim]-0.3
sim_height2 = fake_linear_decimated[idx_sim]
lin_height =  simulated_step_truelinear[idx_lin, 1][0]-0.3
print(idx_lin[0].shape)
print('shape mea: ', measured_height.shape)
print('shape non: ', sim_height.shape)
print('shape fake: ', sim_height2.shape)
print('shape lin: ', lin_height.shape)


##
from sklearn.metrics import mean_squared_error
rmse1 = mean_squared_error(measured_height, sim_height, squared=False)/(measured_height.max()-measured_height.min())
rmse2 = mean_squared_error(measured_height, sim_height2, squared=False)/(measured_height.max()-measured_height.min())
rmse3 = mean_squared_error(measured_height, lin_height, squared=False)/(measured_height.max()-measured_height.min())
# normalized rmse
plt.figure()
# plt.plot(tt[idx], simulated_step_non_linear[idx, 3][0], '-')
plt.plot(t[idx_meas]- start_t2, measured_step4[idx_meas, index_z_height][0]-0.316, label=r'$z_{Measured}$(m): Real data on May30th')
plt.plot(tt_new[idx_sim]- time_z_step, non_linear_decimated[idx_sim]-0.3,'--', label=r'$z_{Non-linear\,1}$(m): NonLinear model with bad battery correction')
plt.plot(tt_new[idx_sim]- time_z_step, fake_linear_decimated[idx_sim],'--', label=r'$z_{Non-linear\,2}$(m): Linearized model with NonLinear elements')
plt.plot(t_lin[idx_lin]- time_z_step_lin, simulated_step_truelinear[idx_lin, 1][0]-0.3,'--', label= r'$z_{Linear}$(m): True linear model')
plt.text(6, 0.8, 'NRMSE of NonLinear 1: ' + str(f'{rmse1:.3}'))
plt.text(6, 0.6, 'NRMSE of NonLinear 2: ' +  str(f'{rmse2:.3}'))
plt.text(6, 0.4, 'NRMSE of TrueLinear: ' +  str(f'{rmse3:.3}'))

plt.xlabel('Time, Seconds')
plt.legend(loc='best')
plt.title('Measured versus Simulation Step Response')
plt.grid()
plt.show()

##
from sklearn.metrics import mean_squared_error
rmse1 = mean_squared_error(measured_height, sim_height, squared=False)/(measured_height.max()-measured_height.min())
rmse2 = mean_squared_error(measured_height, sim_height2, squared=False)/(measured_height.max()-measured_height.min())
rmse3 = mean_squared_error(measured_height, lin_height, squared=False)/(measured_height.max()-measured_height.min())

print(rmse1)
print(rmse2)
print(rmse3)
