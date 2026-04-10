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
        # self.set_opti(N_horizon=1)

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


class BackAndForthNStepControlInvariance(ControlInvarianceOCP):
    def __init__(self, model):
        super().__init__(model)
        self.r_back = 0
        self.j_forward = self.params.N 
        self.set_opti()

    # set base ocp (only dynamics and obstacles/bounds constraints)
    def set_opti(self):
        opti = cs.Opti()
        # x_init = opti.parameter(self.model.nx)
        cost = 0

        # Define decision variables X0 + X*r_back + X*j_forward
        X, U = [], []
        X += [opti.variable(self.model.nx)]
        # opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
        for k in range(self.r_back + self.j_forward):
            X += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
            U += [opti.variable(self.model.nu)]

        # backward phase constraints
        for k in range(self.r_back):
            opti.subject_to(X[k + 1] == self.model.f_fun_back(X[k], U[k]))
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
        
        # forward phase constraints
        for k in range(self.r_back, self.r_back + self.j_forward + 1):
            if k < self.r_back + self.j_forward:
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
            ((self.r_back + self.j_forward) + 1, self.model.nq * 2)
        )  # (N+1) x nq *2
        self.ug = np.zeros(((self.r_back + self.j_forward), self.model.nu))  #  (N) x nq

        self.additionalSetting()
        self.opti.minimize(self.cost)

    def set_r_back(self, r):
        self.r_back = r

    def set_j_forward(self, j_forward):
        self.j_forward = j_forward

    def solveProblem(self, x_init, r_back, j_forward):
        self.set_r_back(r_back)
        self.set_j_forward(j_forward)
        self.set_opti()
        self.setXBoundaryConditions(x_init, 0)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            return True, sol
        except:
            return False, None
class BackAndForthNStepControlInvarianceAlternative(ControlInvarianceOCP):
    def __init__(self, model):
        super().__init__(model)
        self.r_back = 0
        self.j_forward = self.params.N 
        self.set_opti()

    # set base ocp (only dynamics and obstacles/bounds constraints)
    def set_opti(self):
        opti = cs.Opti()
        # x_init = opti.parameter(self.model.nx)
        cost = 0

        # There will be two shooting trajectories, one for the backward phase and one for the forward phase, with free initial condition
        # "backward" trajectory
        X_back, U_back = [], []
        X_back += [opti.variable(self.model.nx)]
        # opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
        for k in range(self.r_back):
            X_back += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X_back[-1], self.model.x_max))
            U_back += [opti.variable(self.model.nu)]

        # backward phase constraints
        for k in range(self.r_back + 1):
            if k < self.r_back:
                opti.subject_to(X_back[k + 1] == self.model.f_fun(X_back[k], U_back[k]))
                # Torque constraints
                opti.subject_to(
                    opti.bounded(
                        self.model.tau_min,
                        self.model.tau_fun(X_back[k], U_back[k]),
                        self.model.tau_max,
                    )
                )
            for bounds, constr in zip(
                self.model.NL_external[-1], self.model.collisions_constr_fun
            ):
                opti.subject_to(opti.bounded(bounds[1], constr[0](X_back[k]), bounds[2]))
        
        # "forward" trajectory
        X_forward, U_forward = [], []
        X_forward += [opti.variable(self.model.nx)]
        # opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
        for k in range(self.j_forward):
            X_forward += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X_forward[-1], self.model.x_max))
            U_forward += [opti.variable(self.model.nu)]

        # forward phase constraints
        for k in range(self.j_forward + 1):
            if k < self.j_forward:
                opti.subject_to(X_forward[k + 1] == self.model.f_fun(X_forward[k], U_forward[k]))
                # Torque constraints
                opti.subject_to(
                    opti.bounded(
                        self.model.tau_min,
                        self.model.tau_fun(X_forward[k], U_forward[k]),
                        self.model.tau_max,
                    )
                )
            for bounds, constr in zip(
                self.model.NL_external[-1], self.model.collisions_constr_fun
            ):
                opti.subject_to(opti.bounded(bounds[1], constr[0](X_forward[k]), bounds[2]))

        # initial state is optimized, but it has to be the same for both trajectories
        opti.subject_to(X_back[0] == X_forward[0])

        self.opti = opti
        self.X_back = X_back
        self.U_back = U_back
        self.X_forward = X_forward
        self.U_forward = U_forward
        self.cost = cost


        self.additionalSetting()
        self.opti.minimize(self.cost)

    def set_r_back(self, r):
        self.r_back = r

    def set_j_forward(self, j_forward):
        self.j_forward = j_forward

    def solveProblem(self, x_init, r_back, j_forward):
        self.set_r_back(r_back)
        self.set_j_forward(j_forward)
        self.set_opti()
        self.setXBoundaryConditions(x_init, r_back)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            return True, sol
        except:
            return False, None

    def additionalSetting(self):
        safe_set_funs = self.safe_set.get_constraints_fun()
        safe_set_bounds = self.safe_set.get_bounds()
        for i, func in enumerate(safe_set_funs):
            self.opti.subject_to(
                self.opti.bounded(
                    safe_set_bounds[i][0], func(self.X_forward[-1]), safe_set_bounds[i][1]
                )
            )

    def setXBoundaryConditions(self, x_init, node):
        if self.opti == None:
            print("error problem not istantiated yet")
            exit()
        else:
            self.opti.subject_to(self.X_back[node] == x_init)

class NStepControlInvarianceOCP_MAX(BaseOCP):
    def __init__(self, model):
        super().__init__(model)
        # self.create_safe_set()

    # set base ocp (only dynamics and obstacles/bounds constraints)
    def set_opti(self, N_horizon):
        opti = cs.Opti()
        x_init = opti.parameter(self.model.nx)

        # Define decision variables
        X, U = [], []
        X += [opti.variable(self.model.nx)]
        opti.subject_to(X[-1][:self.model.nq]==x_init[:self.model.nq])
        for k in range(N_horizon):
            X += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
            U += [opti.variable(self.model.nu)]

        for k in range(N_horizon + 1):

            if k < N_horizon:
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
        self.x_init = x_init
        vel_direction = x_init[self.model.nq:]/cs.norm_2(x_init[self.model.nq:])
        cost =  - vel_direction.T @ (X[0][self.model.nq:]) 
        self.cost = cost
        self.xg = np.zeros(
            (self.model.params.N + 1, self.model.nq * 2)
        )  # (N+1) x nq *2
        self.ug = np.zeros((self.model.params.N, self.model.nu))  #  (N) x nq
        self.opti.minimize(self.cost)

    def setXBoundaryConditions(self, x_init):
        if self.opti == None:
            print("error problem not istantiated yet")
            exit()
        else:
            self.opti.set_value(self.x_init, x_init)
            direction = x_init[self.model.nq:]/cs.norm_2(x_init[self.model.nq:])
            self.opti.subject_to(direction.T @ self.X[0][self.model.nq:] <=  direction.T @ x_init[self.model.nq:])
    
    def solveProblem(self, x_init, horizon):
        self.set_opti(horizon)
        self.setXBoundaryConditions(x_init)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            result = ([True, sol.value(self.X)[0]])
        except:
            result = ([False, None])
        return result
    
    # def create_safe_set(self):
    #     # Analytic or network set
    #     if self.model.params.use_net == True:
    #         self.safe_set = NetSafeSet(self.model, cs.MX.sym("p", 5))
    #         self.net_name = "_net"
    #     elif self.model.params.use_net == False:
    #         self.safe_set = AnalyticSafeSet(self.model, cs.MX.sym("p", 5))
    #         self.net_name = "analytic_set"
    

class BackAndForthNStepControlInvariance_MAX(NStepControlInvarianceOCP_MAX):
    def __init__(self, model):
        super().__init__(model)

    # set base ocp (only dynamics and obstacles/bounds constraints)
    def set_opti(self):
        opti = cs.Opti()
        x_init = opti.parameter(self.model.nx)

        # Define decision variables X0 + X*r_back + X*j_forward
        X, U = [], []
        X += [opti.variable(self.model.nx)]
        opti.subject_to(X[-1][:self.model.nq]==x_init[:self.model.nq])
        for k in range(self.r_back + self.j_forward):
            X += [opti.variable(self.model.nx)]
            opti.subject_to(opti.bounded(self.model.x_min, X[-1], self.model.x_max))
            U += [opti.variable(self.model.nu)]

        # backward phase constraints
        for k in range(self.r_back):
            opti.subject_to(X[k + 1] == self.model.f_fun_back(X[k], U[k]))
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
        
        # forward phase constraints
        for k in range(self.r_back, self.r_back + self.j_forward + 1):
            if k < self.r_back + self.j_forward:
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
        self.x_init = x_init
        vel_direction = x_init[self.model.nq:]/cs.norm_2(x_init[self.model.nq:])
        cost =  - vel_direction.T @ (X[0][self.model.nq:]) 
        self.cost = cost
        self.xg = np.zeros(
            ((self.r_back + self.j_forward) + 1, self.model.nq * 2)
        )  # (N+1) x nq *2
        self.ug = np.zeros(((self.r_back + self.j_forward), self.model.nu))  #  (N) x nq
        self.opti.minimize(self.cost)

    def set_r_back(self, r):
        self.r_back = r

    def set_j_forward(self, j_forward):
        self.j_forward = j_forward

    def solveProblem(self, x_init, r_back, j_forward):
        self.set_r_back(r_back)
        self.set_j_forward(j_forward)
        self.set_opti()
        self.setXBoundaryConditions(x_init, 0)
        self.ocp = self.instantiateProblem()
        try:
            sol = self.opti.solve()
            return True, sol
        except:
            return False, None