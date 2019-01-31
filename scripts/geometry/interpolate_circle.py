import numpy as np
import matplotlib.pyplot as plt


t = 1*np.exp(np.linspace(0,2*np.pi,100)*1j)
plt.plot(np.real(t), np.imag(t))

tstart = np.pi/2.0
tgoal = 2*np.pi
tstart_proj = np.exp(tstart*1j)
tgoal_proj = np.exp(tgoal*1j)
plt.plot(np.real(tstart_proj), np.imag(tstart_proj),'og')
plt.plot(np.real(tgoal_proj), np.imag(tgoal_proj),'or')

##interpolate
s = np.linspace(0,4,100)
m = (np.floor(s)/10+1.05)*np.exp((tstart + s*(tgoal - tstart))*1j)
plt.plot(np.real(m), np.imag(m),'-m')

plt.show()
# plt.plot(tstart, tend)
# t = np.linspace(0,2*np.pi,100)
# plt.plot(t, np.exp(j*t))




