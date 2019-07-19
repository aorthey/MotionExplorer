import numpy as np
import re

def state2vec(sstr):
    numeric_const_pattern = '(?<!SO)[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )'
    rx = re.compile(numeric_const_pattern, re.VERBOSE)
    res = rx.findall(sstr)

    res = np.array(res)
    res = res.astype(float)
    res = res.reshape(-1,2)
    st = res[:,0]
    sx = res[:,1]

    stStr = ','.join(['%.6f' % num for num in st])
    sxStr = ','.join(['%.6f' % num for num in sx])

    # print(sstr)
    print("#"*80)
    print("std::vector<double> s1t{"+stStr+"};")
    print("std::vector<double> s1x{"+sxStr+"};")

sstr1="""
Compound state [
    SO2State [0]
    RealVectorState [0]
    ]
Compound state [
    SO2State [-1.07171]
    RealVectorState [-1.07812]
    ]
Compound state [
    SO2State [-1.09713]
    RealVectorState [-1.08255]
    ]
Compound state [
    SO2State [-1.11643]
    RealVectorState [-1.07635]
    ]
Compound state [
    SO2State [-1.13823]
    RealVectorState [-1.06704]
    ]
Compound state [
    SO2State [-1.16503]
    RealVectorState [-1.0515]
    ]
Compound state [
    SO2State [-1.1977]
    RealVectorState [-1.02873]
    ]
Compound state [
    SO2State [-3.14]
    RealVectorState [0.8]
    ]
"""
sstr2="""
Compound state [
    SO2State [0]
    RealVectorState [0]
    ]
Compound state [
    SO2State [-1.01496]
    RealVectorState [-1.03935]
    ]
Compound state [
    SO2State [-1.08578]
    RealVectorState [-1.0906]
    ]
Compound state [
    SO2State [-1.14566]
    RealVectorState [-1.07955]
    ]
Compound state [
    SO2State [-3.14]
    RealVectorState [0.8]
    ]
"""
sstr3="""
Compound state [
    SO2State [0]
    RealVectorState [0]
    ]
Compound state [
    SO2State [-1.07886]
    RealVectorState [-1.18716]
    ]
Compound state [
    SO2State [-3.14]
    RealVectorState [0.8]
    ]
"""
sstr4="""
Compound state [
    SO2State [0]
    RealVectorState [0]
    ]
Compound state [
    SO2State [-1.07294]
    RealVectorState [-1.08217]
    ]
Compound state [
    SO2State [-1.1036]
    RealVectorState [-1.08527]
    ]
Compound state [
    SO2State [-1.14177]
    RealVectorState [-1.06693]
    ]
Compound state [
    SO2State [-1.2117]
    RealVectorState [-1.01782]
    ]
Compound state [
    SO2State [-3.14]
    RealVectorState [0.8]
    ]
"""


state2vec(sstr1)
state2vec(sstr2)
state2vec(sstr3)
state2vec(sstr4)

