import numpy as np
import re

def state2vec(sstr):
    numeric_const_pattern = '(?<!SO)[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )'
    rx = re.compile(numeric_const_pattern, re.VERBOSE)
    res = rx.findall(sstr)

    res = np.array(res)
    res = res.astype(float)
    res = res.reshape(-1,2)
    sx = res[:,0]
    sy = res[:,1]

    sxStr = ','.join(['%.6f' % num for num in sx])
    syStr = ','.join(['%.6f' % num for num in sy])

    # print(sstr)
    print("#"*80)
    print("std::vector<double> sx{"+sxStr+"};")
    print("std::vector<double> sy{"+syStr+"};")

sstr1="""
RealVectorState [-2 0]
RealVectorState [-0.512838 -0.436827]
RealVectorState [0.442596 -0.452332]
RealVectorState [0.474337 -0.445104]
RealVectorState [0.509194 -0.435831]
RealVectorState [2 -0]
"""
sstr2="""
RealVectorState [-2 0]
RealVectorState [-0.768499 -0.380547]
RealVectorState [-0.491926 -0.43781]
RealVectorState [0.432292 -0.458197]
RealVectorState [1.40463 -0.213621]
RealVectorState [2 -0]
"""
sstr3="""
RealVectorState [-2 0]
RealVectorState [-0.573877 0.434022]
RealVectorState [0.363747 0.50796]
RealVectorState [2 -0]
"""
sstr4="""
RealVectorState [-2 0]
RealVectorState [-0.50234 0.437578]
RealVectorState [0.492229 0.437748]
RealVectorState [0.517778 0.433951]
RealVectorState [2 -0]
"""

# res = re.findall(r"[-+]?\d*\.\d+|\d+", sstr)
state2vec(sstr1)
state2vec(sstr2)
state2vec(sstr3)
state2vec(sstr4)

