import numpy as np
from scipy import optimize

# code created from https://stackoverflow.com/questions/55800584/is-there-any-quadratic-programming-function-that-can-have-both-lower-and-upper-b


class quadprog(object):

    def __init__(self,H,f,A, b, x0, lb, ub,bPartialX):
        self.H = H
        self.f = f
        self.A = A
        self.b = b
        self.x0 = x0
        self.bPartialX = bPartialX
        self.bnds = tuple([(lb[x], ub[x]) for x in range(np.size(x0))])
        # call solver
        self.result = self.solver()

    def objective_function(self, x):
        return 0.5*np.dot(np.dot(x.T, self.H), x) + np.dot(self.f.T, x)

    def solver(self):
        cons = ({'type': 'ineq', 'fun': lambda x: self.b - np.dot(-1 * np.dot(np.array(self.bPartialX), np.array([[np.cos(x[2]),0,0,0,0],[np.sin(x[2]),0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1]])), x)})
        optimum = optimize.minimize(self.objective_function,
                                    x0          = self.x0.T,
                                    bounds      = self.bnds,
                                    constraints = cons,
                                    tol         = 10**-3)
        return optimum