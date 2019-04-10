import numpy as np
import cvxpy as cvx

def GetBasisFunctionsAtSamples(X):
  print X.shape
  if len(X.shape) > 1:
    N_dim = int(X.shape[0])
    M_samples = X.shape[1]
  else:
    N_dim = 1
    M_samples = X.shape[0]

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
  ThetaUniform[:,-1]=0

  if N_dim>1:
    D = np.zeros((N_dim, K_basic_functions, M_samples))
  else:
    D = np.zeros((K_basic_functions, M_samples))

  for k in range(0,M_samples):
    for i in range(0,K_basic_functions):
      if N_dim>1:
        D[:,i,k] = ThetaUniform[:,i]
      else:
        D[i,k] = ThetaUniform[:,i]


  return D

if __name__ == '__main__':
  X = np.array([0.1,0.2,0.3,0.4])
  Y = np.array([0.4,0.4,0.4,0.4])
  
  Phi = GetBasisFunctionsAtSamples(X)
  K_basis_functions = Phi.shape[0]
  M_samples = Phi.shape[1]

  W = cvx.Variable((K_basis_functions))

  obj = cvx.Minimize(cvx.norm(Phi.T*W - Y))
  constr = [W>=0, np.ones(K_basis_functions).T*W == 1]
  prob = cvx.Problem(obj, constr)
  prob.solve()

  W = W.value
  print W
