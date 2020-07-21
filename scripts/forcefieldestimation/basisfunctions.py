import numpy as np

def GetBasisFunctionsAtSamples(X):
  N_dim = int(X.shape[0])
  M_samples = X.shape[1]
  K_basic_functions = N_dim+1

  #
  # For each sample we compute matrix
  #
  # Ndim
  #  |
  #  |
  #  |_______ Basis Functions
  #
  ThetaUniform= np.eye(N_dim, K_basic_functions)
  ThetaUniform[0,2]=1
  ThetaUniform[1,2]=1

  D = np.zeros((N_dim, K_basic_functions, M_samples))

  for k in range(0,M_samples):
    for i in range(0,K_basic_functions):
      D[:,i,k] = ThetaUniform[:,i]

  return D

#def GetBasisFunctionsAtSamples( Fsamples ):
#  ##### radial basis functions
#  N=5
#  ll=0
#  ul=1

#  ux = uy = np.linspace(-1,1,10)
#  sux,suy = np.meshgrid(ux,uy)

#  sx = sy = np.linspace(ll, ul, N)
#  p = np.linspace(-1,1,5)
#  ssx,ssy,pp = np.meshgrid(sx,sy,p)

#  ssx= np.reshape(ssx,ssx.shape[0]*ssx.shape[1]*ssx.shape[2],[])
#  ssy= np.reshape(ssy,ssy.shape[0]*ssy.shape[1]*ssy.shape[2],[])
#  spp= np.reshape(pp,pp.shape[0]*pp.shape[1]*pp.shape[2],[])
#  sux= np.reshape(sux,sux.shape[0]*sux.shape[1],[])
#  suy= np.reshape(suy,suy.shape[0]*suy.shape[1],[])

#  ModelRadial = np.dstack([ssx,ssy,spp])[0]
#  ModelUniform = np.dstack([sux,suy])[0]

#  K = ModelRadial.shape[0]
#  M = Fsamples.shape[0]
#  ThetaRadial = np.zeros([K,M,2])
#  for i in range(0,K):
#    #print i,"/",K
#    for j in range(0,M):
#      ix = ModelRadial[i,0]
#      iy = ModelRadial[i,1]
#      ip = ModelRadial[i,2]
#      px = Fsamples[j,0]
#      py = Fsamples[j,1]

#      pp = np.array([px,py])
#      ss = np.array([ix,iy])
    
#      r = np.linalg.norm(pp-ss)

#      if r > 0:
#        ThetaRadial[i,j,:] = ip*(pp-ss)/r
#      else:
#        ThetaRadial[i,j,:] = 0

#  K = ModelUniform.shape[0]
#  M = Fsamples.shape[0]
#  ThetaUniform= np.zeros([K,M,2])
#  for i in range(0,K):
#    #print i,"/",K
#    for j in range(0,M):
#      ix = ModelUniform[i,0]
#      iy = ModelUniform[i,1]
#      ss = np.array([ix,iy])
#      ThetaUniform[i,j,:] = ss

#  FF = Fsamples[:,2:4]
#  Kr = ModelRadial.shape[0]
#  Ku = ModelUniform.shape[0]

#  D = np.zeros([Kr+Ku,2])

#  print Fsamples.shape
#  print ThetaRadial.shape
#  for i in range(0,Kr):
#    D[i,:] = ThetaRadial[i,:,:]
#    #np.linalg.norm(FF - ThetaRadial[i,:,:]) 
#  for i in range(Kr,Kr+Ku):
#    D[i,:] = ThetaUniform[i-Kr,:,:]
#    #D[i] = np.linalg.norm(FF - ThetaUniform[i-Kr,:,:]) 

#  #idx = np.argmin(D)
#  return D

