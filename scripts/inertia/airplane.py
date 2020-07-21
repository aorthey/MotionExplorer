import numpy as np

#length/radius of model
l = 2.32
r = 0.35


#length/radius of original
m_unscaled = 61688 #takeoff weight airbus A380 (lower limit)
l_unscaled = 31
r_unscaled = r/l * l_unscaled

rho_density = np.pi*r_unscaled*r_unscaled*l_unscaled/m_unscaled

m = rho_density*np.pi*r*r*l

ixx = 0
iyy = (1.0/12)*m*l*l
izz = (1.0/12)*m*l*l
print('<inertial>')
print('<mass value=\"'+str(m)+'\"/>')
print('<inertia ixx=\"'+str(ixx)+\
    '\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"'+str(iyy)+\
    '\" iyz=\"0.0\" izz=\"'+str(izz)+'\"/>')
print('</inertial>')

