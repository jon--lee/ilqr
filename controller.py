from scipy.linalg import inv
import numpy as np
from numpy import dot



def mul(*args):
    res = args[0]
    for arg in args[1:]:
        res = np.dot(res, arg)
    return res

def breakup_C(C, n):
    c1 = C[:n, :n]
    c2 = C[:n, n:]
    c3 = C[n:, :n]
    c4 = C[n:, n:]
    return c1, c2, c3, c4


def breakup_c(c, n):
    return c[:n], c[n:]





class LQController():

    def __init__(self, T, n, d, Cs, Fs, cs, fs):
        self.n = n
        self.d = d
        self.backups( T,Cs, Fs, cs, fs)
    

    def backups(self, T, Cs, Fs, cs, fs):
        C1, C2, C3, C4 = breakup_C(Cs[-1], self.n)
        c1, c2 = breakup_c(cs[-1], self.n)

        K_T = - dot( inv(C4), C3)
        k_T = - dot( inv(C4), c2)
        

        V_T = C1 + mul(C2, K_T) + mul(K_T.T, C3) + mul(K_T.T, C4, K_T)
        v_T = c1 + mul(C2, k_T) + mul(K_T.T, c2) + mul(K_T.T, C4, k_T)

        Vs, vs, Ks, ks = [None] * T, [None] * T, [None] * T, [None] * T
        Vs[-1], vs[-1], Ks[-1], ks[-1] = V_T, v_T, K_T, k_T

        for t in range(T - 1)[::-1]:
            C_t = Cs[t]
            F_t = Fs[t]
            c_t = cs[t]
            f_t = fs[t]

            Q_t = C_t + mul(F_t.T, Vs[t + 1], F_t)
            q_t = c_t + mul(F_t.T, Vs[t + 1], f_t) + mul(F_t.T, vs[t + 1])


            Q1, Q2, Q3, Q4 = breakup_C(Q_t, self.n)
            q1, q2 = breakup_c(q_t, self.n)
            Ks[t] = - mul(inv(Q4), Q3)
            ks[t] = - mul(inv(Q4), q2)
            Vs[t] = Q1 + mul(Q2, Ks[t]) + mul(Ks[t].T, Q3) + mul(Ks[t].T, Q4, Ks[t])
            vs[t] = q1 + mul(Q2, ks[t]) + mul(Ks[t].T, q2) + mul(Ks[t].T, Q4, ks[t])

        self.Vs = Vs
        self.vs = vs
        self.Ks = Ks
        self.ks = ks

    def control(self, x, t, x_t = 0, u_t = 0):
        contr = mul(self.Ks[t], x - x_t) + self.ks[t] + u_t
        return contr








if __name__ == '__main__':
    T = 50
    n = 2
    d = 1
    Fs = [np.array([[1, 1, 0], [0, 1, 1]])] * T
    Cs = [np.array([[1, 0, 0], 
                    [0, 1, 0],
                    [0, 0, 1]])] * T

    fs = [np.zeros(n)] * T
    cs = [np.zeros(n + d)] * T

    contr = LQController(T, n, d, Cs, Fs, cs, fs)
    x = np.array([15, 25])
    for t in range(T - 1):
        print x
        u = contr.control(x, t)
        print u
        x = mul(Fs[t], np.hstack((x, u))) + fs[t]

    print x









