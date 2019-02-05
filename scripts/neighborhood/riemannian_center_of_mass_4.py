import sys
import numpy as np
import matplotlib.pyplot as plt

import mpl_toolkits.mplot3d.axes3d as p3

fs = 30
def PlotCircle(ax, x, y, radius, borderadd=0):
  circle1 = plt.Circle((x, y), radius-radius/50.0-borderadd, color='w')
  circle2 = plt.Circle((x, y), radius, color='k')
  ax.add_patch(circle2)
  ax.add_patch(circle1)
  plt.plot(x,y, 'ok')
  plt.plot([0,x],[0,y],'--k')

fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot

xc = np.sqrt(2)/2.0
t2 = np.pi/4.0
t3 = 2.5*np.pi/4.0
t4 = 3.6*np.pi/4.0

x2,y2 = np.cos(t2),np.sin(t2)
x3,y3 = np.cos(t3),np.sin(t3)
x4,y4 = np.cos(t4),np.sin(t4)

PlotCircle(ax, x2, y2, 0.8)
PlotCircle(ax, x3, y3, 0.8)
PlotCircle(ax, x4, y4, 0.3, borderadd=0.01)
PlotCircle(ax, 0, 0, 1)

tout = (t2+t3+t4)/3.0
xc, yc = np.cos(tout),np.sin(tout)
xout, yout = -xc, -yc

arrowOutward = plt.arrow(xout, yout, xout, yout, width=0.05, color='k', length_includes_head=True)
arrowMean = plt.arrow(0,0,xc,yc, width=0.03, color='k',
    length_includes_head=True, capstyle='round')

annotation_string = r"$x_{RCoM}$"
plt.annotate(annotation_string, xy=(0.6*xc+0.05, 0.65*yc), fontsize=0.8*fs)
annotation_string = r"$x_{center}$"
plt.annotate(annotation_string, xy=(-0.2,-0.15), fontsize=0.8*fs)
annotation_string = r"$x_{1}$"
plt.annotate(annotation_string, xy=(x2, y2+0.1), fontsize=0.8*fs)
annotation_string = r"$x_{2}$"
plt.annotate(annotation_string, xy=(x3-0.1, y3+0.1), fontsize=0.8*fs)
annotation_string = r"$x_{3}$"
plt.annotate(annotation_string, xy=(x4-0.2, y4+0.05), fontsize=0.8*fs)
annotation_string = r"$x_{out}$"
plt.annotate(annotation_string, xy=(xout+0.7*xout, yout+0.4*yout), fontsize=0.8*fs)

ax.add_patch(arrowOutward)
ax.add_patch(arrowMean)

ax.autoscale_view()

ax.set_xlabel('X',fontsize=fs)
ax.set_ylabel('Y', fontsize=fs, rotation=np.pi/2.0)

plt.xticks(fontsize=0.7*fs)
plt.yticks(fontsize=0.7*fs)


plt.axis('equal')

plt.show()

#print "Riemannian Mean: ",q

