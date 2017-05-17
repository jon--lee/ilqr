import gym
import scipy.optimize
import numpy as np
import IPython
from controller import LQController

"""
    Turn off the control clipping in OpenAI gym
    in order for this to work
"""

sim = gym.envs.make('Pendulum-v0')

def finite_diff1(x, f):
    fprime = scipy.optimize.approx_fprime
    fx = f(x)
    def g(x, i):
        return f(x)[i]

    if isinstance(fx, float):
        return fprime(x, f, 1e-5)

    A = np.zeros((len(fx), len(x)))
    for i in range(len(fx)):
        A[i, :] = fprime(x, g, 1e-5, i)

    return A

def finite_diff2(x, f):
    fprime = scipy.optimize.approx_fprime
    Q = np.zeros((len(x), len(x)))
    def g(x, i):
        res = fprime(x, f, 1e-5)
        return res[i]
    for i in range(len(x)):
        Q[i, :] = fprime(x, g, 1e-5, i)
    return Q


def reset(env):
    env.reset()
    env.env.state = np.array([np.pi, 0.0])
    return env.env.state

def forward_prop(env, u_array, T):
    reset(env)
    x_array = [env.env.state]

    for t in range(T - 1):
        x, c = step(env, u_array[t])
        x_array.append(x)
        # env.render()

    return np.array(x_array)

def simulate(xu):
    sim.reset()
    sim.env.state = xu[:2]
    x, c = step(sim, xu[2:])
    return x, c

def simulate_x(xu):
    return simulate(xu)[0]

def simulate_c(xu):
    return simulate(xu)[1]

def approx(env, x_array, u_array, T):
    Cs = []
    Fs = []
    cs = []
    fs = []
    for t in range(T - 1):
        x, u = x_array[t], u_array[t]
        xu = np.hstack((x, u))
        C = finite_diff2(xu, simulate_c)
        F = finite_diff1(xu, simulate_x)
        c = finite_diff1(xu, simulate_c)
        f = np.zeros(x.shape)

        Cs.append(C)
        Fs.append(F)
        cs.append(c)
        fs.append(f)

    x = x_array[-1]
    u = np.zeros(u.shape)
    xu = np.hstack((x, u))
    Cs.append(finite_diff2(xu, simulate_c))
    cs.append(finite_diff1(xu, simulate_c))
    Fs.append(np.zeros(Fs[0].shape))
    fs.append(np.zeros(x.shape))

    return Cs, Fs, cs, fs


def step(env, u):
    x, r, _, _ = env.step(u)
    x = env.env.state
    return x,  10* np.dot(x, x) + .3 * u.dot(u)


if __name__ == '__main__':

    env = gym.envs.make('Pendulum-v0')

    T = 400

    x = np.array([.4, 2])
    u = np.array([.3])

    u_array = np.random.normal(0, 1, (T - 1, 1))
    x_array = forward_prop(env, u_array, T)

    for i in range(40):
        Cs, Fs, cs, fs = approx(env, x_array, u_array, T)
        contr = LQController(T, 2, 1, Cs, Fs, cs, fs)

        x = reset(env)
        u_hat_array = [None] * (T - 1)
        x_hat_array = [None] * T

        cost = 0.0

        for t in range(T - 1):
            if i % 5 == 0:
                env.render()
            u = contr.control(x, t, x_t = x_array[t], u_t = u_array[t])
            x_hat_array[t] = x
            u_hat_array[t] = u

            x, c = step(env, u)
            cost += c

        x_hat_array[-1] = x

        x_array = np.array(x_hat_array)
        u_array = np.array(u_hat_array)

        print "cost:", cost



