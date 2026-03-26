import numpy as np
import casadi as cs
from copy import deepcopy
from .safe_set import NetSafeSet, AnalyticSafeSet


class BaseOCP:
    """Define OCP problem and solver (IpOpt)"""

    def __init__(self, model):
        self.params = model.params
        self.model = model
        self.ocp = None

    # set base ocp (only dynamics and obstacles/bounds constraints)
    def set_opti(self, N_horizon):
        opti = cs.Opti()
        # x_init = opti.parameter(self.model.nx)
        cost = 0

        # Define decision variables
        X, U = [], []
        X += [opti.variable(self.model.nx)]
        for k in range(N_horizon):
            X += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
            U += [opti.variable(self.model.nu)]

        # opti.subject_to(X[0] == x_init)

        # Q = self.params.Q_weight * np.eye(self.model.ee_ref.shape[0])
        # ee_ref = self.model.ee_ref
        # ee_ref = np.array([0.6, 0.4, 0.15])

        for k in range(N_horizon + 1):

            # ee_pos = self.model.ee_fun(X[k])
            # cost += (ee_pos - ee_ref).T @ Q @ (ee_pos - ee_ref)
            if k < N_horizon:
                # cost += U[k].T @ R @ U[k]
                # Dynamics constraint
                opti.subject_to(X[k + 1] == self.model.f_fun(X[k], U[k]))
                # Torque constraints
                opti.subject_to(
                    opti.bounded(
                        self.model.tau_min,
                        self.model.tau_fun(X[k], U[k]),
                        self.model.tau_max,
                    )
                )

            for bounds, constr in zip(
                self.model.NL_external[-1], self.model.collisions_constr_fun
            ):
                opti.subject_to(opti.bounded(bounds[1], constr[0](X[k]), bounds[2]))

        self.opti = opti
        self.X = X
        self.U = U
        self.cost = cost
        self.xg = np.zeros(
            (self.model.params.N + 1, self.model.nq * 2)
        )  # (N+1) x nq *2
        self.ug = np.zeros((self.model.params.N, self.model.nu))  #  (N) x nq

        self.additionalSetting()
        self.opti.minimize(self.cost)

    # in additional settings set constraint for particular class of set
    def additionalSetting(self):
        self.net_name = ""

    def instantiateProblem(self):
        opti = self.opti
        opti.solver("ipopt", self.model.params.ipopt_opts)
        return opti

    def solveProblem(self, x_init):
        self.setXBoundaryConditions(x_init, 0)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            return True, sol
        except:
            return False, None

    def setXBoundaryConditions(self, x_init, node):
        if self.opti == None:
            print("error problem not istantiated yet")
            exit()
        else:
            self.opti.subject_to(self.X[node] == x_init)
    
    def setNodeSafeSet(self, x_init, node):
        if self.opti == None:
            print("error problem not istantiated yet")
            exit()
        else:
            self.opti.subject_to(self.X[node] == x_init)

class ControlInvarianceOCP(BaseOCP):
    def __init__(self, model):
        super().__init__(model)
        self.create_safe_set()
        self.set_opti(N_horizon=1)

    def create_safe_set(self):
        # Analytic or network set
        if self.model.params.use_net == True:
            self.safe_set = NetSafeSet(self.model, cs.MX.sym("p", 5))
            self.net_name = "_net"
        elif self.model.params.use_net == False:
            self.safe_set = AnalyticSafeSet(self.model, cs.MX.sym("p", 5))
            self.net_name = "analytic_set"

    def additionalSetting(self):
        safe_set_funs = self.safe_set.get_constraints_fun()
        safe_set_bounds = self.safe_set.get_bounds()
        for i, func in enumerate(safe_set_funs):
            self.opti.subject_to(
                self.opti.bounded(
                    safe_set_bounds[i][0], func(self.X[-1]), safe_set_bounds[i][1]
                )
            )
    
    def solveProblem(self, x_init):
        self.set_opti(1)
        self.setXBoundaryConditions(x_init, 0)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            result = ([True, deepcopy(sol)])
        except:
            result = ([False, None])
        return result
    
class NStepControlInvarianceOCP(BaseOCP):
    def __init__(self, model):
        super().__init__(model)
        self.create_safe_set()
        self.set_opti(N_horizon=1)

    def create_safe_set(self):
        # Analytic or network set
        if self.model.params.use_net == True:
            self.safe_set = NetSafeSet(self.model, cs.MX.sym("p", 5))
            self.net_name = "_net"
        elif self.model.params.use_net == False:
            self.safe_set = AnalyticSafeSet(self.model, cs.MX.sym("p", 5))
            self.net_name = "analytic_set"

    def additionalSetting(self):
        safe_set_funs = self.safe_set.get_constraints_fun()
        safe_set_bounds = self.safe_set.get_bounds()
        for i, func in enumerate(safe_set_funs):
            self.opti.subject_to(
                self.opti.bounded(
                    safe_set_bounds[i][0], func(self.X[-1]), safe_set_bounds[i][1]
                )
            )
    
    def solveProblem(self, x_init, horizon):
        self.set_opti(horizon)
        self.setXBoundaryConditions(x_init, 0)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            result = ([True, deepcopy(sol)])
        except:
            result = ([False, None])
        return result


class UpToNStepControlInvariance(ControlInvarianceOCP):
    def __init__(self, model):
        super().__init__(model)

    def solveProblem(self, x_init):
        result_list = []
        for i in range(1, self.params.N + 1):
            self.set_opti(i)
            self.setXBoundaryConditions(x_init, 0)
            self.ocp = self.instantiateProblem()
            try:
                sol = self.opti.solve()
                result_list.append([True, deepcopy(sol)])
            except:
                result_list.append([False, None])
        return result_list


class BackAndForthNStepControlInvariance(BaseOCP):
    def __init__(self, model):
        super().__init__(model)
        self.r = self.params.N - 1

    def set_r_back_and_forth(self, r):
        self.r = r

    def solveProblem(self, x_init):
        result_list = []
        for i in range(
            0, self.params.N
        ):  # N=10 at i=N-2=8 ocp_length=2 r=1 , N=10 i=0 length=10 r=9
            self.set_opti(self.params.N)
            self.set_r_back_and_forth(i)
            self.setXBoundaryConditions(x_init, self.r)
            self.ocp = self.instantiateProblem()
            try:
                sol = self.opti.solve()
                result_list.append([True, deepcopy(sol)])
            except:
                result_list.append([False, None])
        return result_list

class BackAndForthWithinNStepControlInvariance(BaseOCP):
    def __init__(self, model):
        super().__init__(model)

    def solveProblem(self, x_init):
        for i in range(0, self.params.N):  # N=10 r=5 i have to try for r=6...10
            for j in range(i+1, self.params.N + 1):
                self.set_opti(j)
                self.set_r_back_and_forth(i)
                self.setXBoundaryConditions(x_init, self.r)
                self.ocp = self.instantiateProblem()
                try:
                    sol = self.opti.solve()
                    return True, sol
                except:
                    return False, None
