import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from decimal import Decimal


from scipy.optimize import curve_fit

x_STRIDE = np.array([3, 4, 5, 6, 7, 8, 9])

time_STRIDE = np.array([0.00115411, 0.0164995, 0.0277467, 0.160775, 21.7163, 67.0978,
  233.493])

x_QRRT = np.array([3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58,
  59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
  78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96,
  97, 98, 99])

time_QRRT = np.array([0.000365832, 0.000402447, 0.000443592, 0.000577693,
  0.000521766, 0.000720844, 0.000769298, 0.000937456, 0.00101675, 0.00128014,
  0.00134447, 0.00151232, 0.00207501, 0.00183388, 0.00215915, 0.00216268,
  0.00247936, 0.00261557, 0.00306443, 0.00337423, 0.00353772, 0.00364211,
  0.00404309, 0.00426356, 0.00455665, 0.00506403, 0.00536069, 0.00595222,
  0.00605241, 0.00653467, 0.0069675, 0.00782163, 0.00768541, 0.00810245,
  0.00857926, 0.00908557, 0.00973545, 0.0101612, 0.0110216, 0.0113805,
  0.0121515, 0.0126515, 0.0135066, 0.0138084, 0.014493, 0.015468, 0.0164111,
  0.0176125, 0.0174337, 0.019029, 0.0190481, 0.0202365, 0.0210693, 0.0220551,
  0.0232046, 0.0237069, 0.0245125, 0.027029, 0.0267159, 0.0284524, 0.0612482,
  0.0293725, 0.0307501, 0.0319048, 0.0703574, 0.0343832, 0.0350723, 0.0364033,
  0.0374535, 0.0387521, 0.0410497, 0.0568186, 0.0425671, 0.0437442, 0.0454531,
  0.0482238, 0.0487631, 0.0494541, 0.0511995, 0.0527107, 0.0539567, 0.0559011,
  0.0573642, 0.0594953, 0.0627771, 0.063916, 0.0637832, 0.0665562, 0.0682659,
  0.0690015, 0.12266, 0.0752062, 0.0757425, 0.0780734, 0.0820006, 0.0829661,
  0.0835596])

x_QRRT = np.array([10, 20, 30, 40, 50, 60, 70, 80, 90, 100, ])

time_QRRT = np.array([0.000809269, 0.00250282, 0.0053551, 0.0096053, 0.0156424,
  0.0237897, 0.0345787, 0.0469422, 0.0621285, 0.0808156, ])

def func(x, a, b, c, d):
      return a*x**3 + b*x**2 +c*x + d

popt_STRIDE, pcov_STRIDE = curve_fit(func, x_STRIDE, time_STRIDE)
popt_QRRT, pcov_QRRT = curve_fit(func, x_QRRT, time_QRRT)

xr=np.linspace(3,100)
xr=np.array(xr, dtype=float) 

plt.yscale('log')


y_STRIDE = func(100, *popt_STRIDE)
y_QRRT = func(100, *popt_QRRT)

plt.plot(xr, func(xr, *popt_STRIDE), color='0.8', linewidth=2, label="STRIDE")
plt.plot(xr, func(xr, *popt_QRRT), color='0.2', linewidth=2, label="QRRT")

plt.plot(x_STRIDE,time_STRIDE,"ok", color='0.8')
plt.plot(x_QRRT,time_QRRT,"ok")

ys = np.exp(0.5*(np.log(y_STRIDE)-np.log(y_QRRT)))

plt.axhline(y=y_STRIDE, color='0.8', linestyle='--')
plt.axhline(y=y_QRRT, color='0.2', linestyle='--')
plt.vlines(100, y_QRRT, y_STRIDE, color='0.2', linestyle='--')

dy = y_STRIDE-y_QRRT
ytext = '%.1E' % Decimal(dy)

plt.text(90, ys, ytext, fontsize=12)

rect = patches.Rectangle((90,ys-1000),15,10000,linewidth=0,edgecolor='r',zorder=2,fill=True, facecolor='white') 

ax = plt.gca()
ax.add_patch(rect)



plt.ylabel("Time (s)")
plt.xlabel("#Dimensions")
plt.legend()

fname_pdf = "../../data/benchmarks/hypercube_STRIDE_QRRT.pdf"
pp = PdfPages(fname_pdf)
plt.tight_layout()
pp.savefig(plt.gcf(), pad_inches=0.0, bbox_inches='tight')
pp.close()

plt.show()
