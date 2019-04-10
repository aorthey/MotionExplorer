import numpy as np
import cvxpy as cvx


def GetBasisFunctionsAtSamples(X):
  N_dim = int(X.shape[0])
  M_samples = X.shape[1]

  K_basic_functions = N_dim
  #
  # For each sample we compute matrix
  #
  # Ndim
  #  |
  #  |
  #  |_______ Basis Functions
  #
  ThetaUniform= np.eye(N_dim, K_basic_functions)

  D = np.zeros((N_dim, K_basic_functions, M_samples))

  for k in range(0,M_samples):
    for i in range(0,K_basic_functions):
      D[:,i,k] = ThetaUniform[:,i]


  return D

if __name__ == '__main__':
  X = np.array(([0.1,0.3],[0.4,0.5]))
  Y = np.array(([0.5,0.8],[0.5,0.8]))

  Phi = GetBasisFunctionsAtSamples(X)
  K_basis_functions = Phi.shape[0]
  M_samples = Phi.shape[1]
  Phi = np.sum(Phi, axis=1)

  W = cvx.Variable((K_basis_functions))


  obj = cvx.Minimize(cvx.norm(W*Phi - Y))
  #constr = [W>=0] #, np.ones(K_basis_functions).T*W == 1]
  prob = cvx.Problem(obj)
  prob.solve()

  W = W.value
  print W
