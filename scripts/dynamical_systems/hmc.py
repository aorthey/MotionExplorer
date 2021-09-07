import numpy as np
import scipy.stats as st
import matplotlib
import matplotlib.pyplot as plt

@np.vectorize
def cost(x,y):
  if (x < -1) or (x > 1) or (y>1) or (y<-1):
    return 0.0
  q = np.array((0.0-x,0.0-y)) 
  d = 1.0/(np.linalg.norm(q)+1)
  if (x < -0.5) or (x > 0.5):
    return d
  elif (y < 0.5) and (y > -0.5):
    return d
  else:
    return 0.0

def costGradient(x,y):
  #derivative of potential function
  return 100*np.array((0.0-x,0.0-y))


def leapfrog(q, p, path_len, step_size):
    """Leapfrog integrator for Hamiltonian Monte Carlo.

    Parameters
    ----------
    q : np.floatX
        Initial position
    p : np.floatX
        Initial momentum
    path_len : float
        How long to integrate for
    step_size : float
        How long each integration step should be

    Returns
    -------
    q, p : np.floatX, np.floatX
        New position and momentum
    """
    q, p = np.copy(q), np.copy(p)

    p -= step_size * costGradient(q[0],q[1]) / 2  # half step
    for _ in range(int(path_len / step_size) - 1):
        q += step_size * p  # whole step
        p -= step_size * costGradient(q[0],q[1])  # whole step
    q += step_size * p  # whole step
    p -= step_size * costGradient(q[0],q[1]) / 2  # half step

    # momentum flip at end
    return q, -p

def hamiltonian_monte_carlo(n_samples, initial_position, path_len=1, step_size=0.5):
    """Run Hamiltonian Monte Carlo sampling.

    Parameters
    ----------
    n_samples : int
        Number of samples to return
    initial_position : np.array
        A place to start sampling from.
    path_len : float
        How long each integration path is. Smaller is faster and more correlated.
    step_size : float
        How long each integration step is. Smaller is slower and more accurate.

    Returns
    -------
    np.array
        Array of length `n_samples`.
    """
    # collect all our samples in a list
    samples = [initial_position]

    # Keep a single object for momentum resampling
    momentum = st.norm(0, 1)

    # If initial_position is a 10d vector and n_samples is 100, we want
    # 100 x 10 momentum draws. We can do this in one call to momentum.rvs, and
    # iterate over rows
    size = (n_samples,) + initial_position.shape[:1]
    for p0 in momentum.rvs(size=size):
        # Integrate over our path to get a new position and momentum
        q_new, p_new = leapfrog(
            samples[-1],
            p0,
            path_len=path_len,
            step_size=step_size,
        )

        # Check Metropolis acceptance criterion
        start_log_p = cost(samples[-1][0],samples[-1][1]) - np.sum(momentum.logpdf(p0))
        new_log_p = cost(q_new[0],q_new[1]) - np.sum(momentum.logpdf(p_new))
        if np.log(np.random.rand()) < start_log_p - new_log_p:
            samples.append(q_new)
        else:
            samples.append(np.copy(samples[-1]))

    return np.array(samples[1:])

initial_position = np.array((-0.5,0.0))
X = hamiltonian_monte_carlo(10000, initial_position, path_len = 0.1, step_size = 0.001)

xx = np.linspace(-2,+2,50)
yy = np.linspace(-2,+2,50)
xv, yv = np.meshgrid(xx, yy)

z = cost(xv,yv)
plt.pcolormesh(xv, yv, z)

ax = plt.gca()

VFa,VFb = costGradient(xv, yv)
plt.quiver(xx, yy, VFa, VFb, color='black', label="Estimated")
# q = ax.quiver(xx, yy, xv, yv)
# ax.quiverkey(q, X=0.3, Y=1.1, U=10, label='Quiver key, length = 10', labelpos='E')

plt.plot(X[:,0],X[:,1], 'ok')
plt.show()
