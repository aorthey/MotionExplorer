import numpy as np
import re

def state2vec(sstr):
    numeric_const_pattern = '(?<!SO)[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )'
    rx = re.compile(numeric_const_pattern, re.VERBOSE)
    res = rx.findall(sstr)

    res = np.array(res)
    res = res.astype(float)
    res = res.reshape(-1,3)
    sx = res[:,0]
    sy = res[:,1]
    st = res[:,2]

    sxStr = ','.join(['%.5f' % num for num in sx])
    syStr = ','.join(['%.5f' % num for num in sy])
    stStr = ','.join(['%.5f' % num for num in st])

    # print(sstr)
    print("#"*80)
    print("std::vector<double> sx{"+sxStr+"};")
    print("std::vector<double> sy{"+syStr+"};")
    print("std::vector<double> st{"+stStr+"};")


sstr1 = """
    Compound state [
    RealVectorState [-2 0]
    SO2State [0]
    ]
    Compound state [
    RealVectorState [-0.729037 -0.420625]
    SO2State [-0.34727]
    ]
    Compound state [
    RealVectorState [-0.534275 -0.489375]
    SO2State [-0.251065]
    ]
    Compound state [
    RealVectorState [-0.347947 -0.513642]
    SO2State [-0.121986]
    ]
    Compound state [
    RealVectorState [0.472544 -0.507627]
    SO2State [0.0935141]
    ]
    Compound state [
    RealVectorState [0.737109 -0.451928]
    SO2State [0.164784]
    ]
    Compound state [
    RealVectorState [2 -0]
    SO2State [0]
    ]"""


sstr2 = """
Compound state [
RealVectorState [-2 0]
SO2State [0]
]
Compound state [
RealVectorState [-0.628456 -0.447904]
SO2State [-0.377566]
]
Compound state [
RealVectorState [-0.446917 -0.496381]
SO2State [-0.171171]
]
Compound state [
RealVectorState [-0.444426 -0.496979]
SO2State [-0.167753]
]
Compound state [
RealVectorState [-0.441936 -0.497577]
SO2State [-0.164334]
]
Compound state [
RealVectorState [-0.355415 -0.508513]
SO2State [-0.104017]
]
Compound state [
RealVectorState [0.373248 -0.513476]
SO2State [-0.0247672]
]
Compound state [
RealVectorState [0.676749 -0.472899]
SO2State [0.101571]
]
Compound state [
RealVectorState [0.73934 -0.451235]
SO2State [0.164809]
]
Compound state [
RealVectorState [2 -0]
SO2State [0]
]"""




sstr3 = """Compound state [
RealVectorState [-2 0]
SO2State [0]
]
Compound state [
RealVectorState [-0.759503 -0.415188]
SO2State [-0.296717]
]
Compound state [
RealVectorState [-0.698055 -0.436257]
SO2State [-0.284035]
]
Compound state [
RealVectorState [-0.685309 -0.44063]
SO2State [-0.27863]
]
Compound state [
RealVectorState [-0.678995 -0.442772]
SO2State [-0.275904]
]
Compound state [
RealVectorState [-0.672681 -0.444915]
SO2State [-0.273177]
]
Compound state [
RealVectorState [-0.666366 -0.447057]
SO2State [-0.270451]
]
Compound state [
RealVectorState [-0.575442 -0.476359]
SO2State [-0.22802]
]
Compound state [
RealVectorState [-0.417259 -0.506541]
SO2State [-0.142121]
]
Compound state [
RealVectorState [-0.365874 -0.50951]
SO2State [-0.119442]
]
Compound state [
RealVectorState [0.233913 -0.509952]
SO2State [0.0777336]
]
Compound state [
RealVectorState [0.347419 -0.507856]
SO2State [0.118539]
]
Compound state [
RealVectorState [0.462833 -0.503666]
SO2State [0.179249]
]
Compound state [
RealVectorState [0.56933 -0.486399]
SO2State [0.278025]
]
Compound state [
RealVectorState [1.53505 -0.161796]
SO2State [0.150305]
]
Compound state [
RealVectorState [2 -0]
SO2State [0]
]"""

sstr4="""
RealVectorState [-2 0]
SO2State [0]
]
Compound state [
RealVectorState [-1.35536 0.243058]
SO2State [0.0353236]
]
Compound state [
RealVectorState [-0.710729 0.486117]
SO2State [0.0706472]
]
Compound state [
RealVectorState [-0.256167 0.527313]
SO2State [-0.0876115]
]
Compound state [
RealVectorState [0.198396 0.56851]
SO2State [-0.24587]
]
Compound state [
RealVectorState [0.743376 0.432368]
SO2State [-0.249987]
]
Compound state [
RealVectorState [2 -0]
SO2State [0]
]"""

sstr5 = """
Compound state [
RealVectorState [-2 0]
SO2State [0]
]
Compound state [
RealVectorState [-0.765982 0.44129]
SO2State [0.173365]
]
Compound state [
RealVectorState [-0.684948 0.466104]
SO2State [0.156585]
]
Compound state [
RealVectorState [-0.453786 0.492073]
SO2State [0.0455057]
]
Compound state [
RealVectorState [0.289157 0.541724]
SO2State [-0.245546]
]
Compound state [
RealVectorState [0.294047 0.541265]
SO2State [-0.246426]
]
Compound state [
RealVectorState [0.298936 0.540806]
SO2State [-0.247306]
]
Compound state [
RealVectorState [0.365039 0.530506]
SO2State [-0.25543]
]
Compound state [
RealVectorState [0.726066 0.432677]
SO2State [-0.260181]
]
Compound state [
RealVectorState [2 -0]
SO2State [0]
]
"""

sstr6 = """
Compound state [
    RealVectorState [-2 0]
    SO2State [0]
    ]
Compound state [
    RealVectorState [-0.48142 -0.508479]
    SO2State [2.75165]
    ]
Compound state [
    RealVectorState [-0.457896 -0.514125]
    SO2State [2.79052]
    ]
Compound state [
    RealVectorState [-0.379435 -0.525622]
    SO2State [2.89891]
    ]
Compound state [
    RealVectorState [-0.303615 -0.528105]
    SO2State [2.95514]
    ]
Compound state [
    RealVectorState [0.352799 -0.511423]
    SO2State [-3.01023]
    ]
Compound state [
    RealVectorState [0.41819 -0.502041]
    SO2State [-2.99079]
    ]
Compound state [
    RealVectorState [0.452892 -0.495742]
    SO2State [-2.96867]
    ]
Compound state [
    RealVectorState [2 -0]
    SO2State [0]
    ]
"""
sstr7="""
Compound state [
    RealVectorState [-2 0]
    SO2State [0]
    ]
Compound state [
    RealVectorState [-0.468902 -0.511855]
    SO2State [2.77296]
    ]
Compound state [
    RealVectorState [-0.419151 -0.520683]
    SO2State [2.8488]
    ]
Compound state [
    RealVectorState [-0.358466 -0.525781]
    SO2State [2.91146]
    ]
Compound state [
    RealVectorState [-0.279461 -0.529141]
    SO2State [2.97442]
    ]
Compound state [
    RealVectorState [0.420236 -0.503321]
    SO2State [-3.00769]
    ]
Compound state [
    RealVectorState [0.436807 -0.500035]
    SO2State [-2.99234]
    ]
Compound state [
    RealVectorState [2 -0]
    SO2State [0]
    ]
"""



# res = re.findall(r"[-+]?\d*\.\d+|\d+", sstr)
state2vec(sstr1)
state2vec(sstr2)
state2vec(sstr3)
state2vec(sstr4)
state2vec(sstr5)
state2vec(sstr6)
state2vec(sstr7)

