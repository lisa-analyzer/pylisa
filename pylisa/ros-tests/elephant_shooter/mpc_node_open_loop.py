import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np
import casadi as ca
from casadi import sin, cos, pi

# setting matrix_weights' variables
V_limit = 40.0
Q_x = 1500
Q_y = 1500
Q_theta = 1000
R1 = 1
R2 = 1
R3 = 1
R4 = 1

step_horizon = 0.1  # time between steps in seconds
N = 10              # number of look ahead steps
wheel_radius = 0.06    # wheel radius
L = 0.285           # L in J Matrix (half robot x-axis length)

# boundary


############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value
#############

def DM2Arr(dm):
    return np.array(dm.full())


class mpc_class(Node):
    def __init__(self):
        super().__init__('mpc_node_test')
        timer_period = 0.1  # seconds
        self.mpc_timer = self.create_timer(timer_period, self.mpc_callback)   ## 10hz
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pub_speed', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mpc_position',
            self.listener_callback,
            10)
        ##### mpc #####
        # specs
        self.x_init = 0
        self.y_init = 0
        self.theta_init = 0
        self.x_target = 0
        self.y_target = 0
        self.theta_target = 0
        ## mpc start
        self.v_max = 5
        self.start_mpc()
        ## 
        self.goal = False
        
        self.V_pub = [0,0,0,0]
        self.x_pos = 0
        self.y_pos = 0
        self.omega_pos = 0

    def listener_callback(self, msg):
        self.x_target = msg.data[0]
        self.y_target = msg.data[1]
        self.theta_target = msg.data[2]
        slow_speed = msg.data[3]
        if slow_speed >= 1.0 :
            self.v_max = 5
        self.state_target = ca.DM([self.x_target, self.y_target, self.theta_target])  # target state
        self.goal = False

    def start_mpc(self):
        # state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )
        self.n_states = states.numel()

        # control symbolic variables
        V_a = ca.SX.sym('V_a')
        V_b = ca.SX.sym('V_b')
        V_c = ca.SX.sym('V_c')
        V_d = ca.SX.sym('V_d')
        controls = ca.vertcat(
            V_a,
            V_b,
            V_c,
            V_d
        )
        self.n_controls = controls.numel()

        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', self.n_states, N + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', self.n_controls, N)

        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', self.n_states + self.n_states)

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_theta)

        # controls weights matrix
        R = ca.diagcat(R1, R2, R3, R4)

        # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )
        # Mecanum wheel transfer function which can be found here: 
        # https://www.researchgate.net/publication/334319114_Model_Predictive_Control_for_a_Mecanum-wheeled_robot_in_Dynamical_Environments
        J = (wheel_radius/(4*0.707)) * ca.DM([
            [         1,         1,          -1,         -1],
            [        1,         -1,          -1,        1],
            [1/(2*L), 1/(2*L), 1/(2*L), 1/(2*L)]
        ])
        # RHS = states + J @ controls * step_horizon  # Euler discretization
        RHS = J @ controls
        # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        self.f = ca.Function('f', [states, controls], [RHS])


        cost_fn = 0  # cost function
        g = X[:, 0] - P[:self.n_states]  # constraints in the equation


        # runge kutta
        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            cost_fn = cost_fn \
                + (st - P[self.n_states:]).T @ Q @ (st - P[self.n_states:]) \
                + con.T @ R @ con
            st_next = X[:, k+1]
            k1 = self.f(st, con)
            k2 = self.f(st + step_horizon/2*k1, con)
            k3 = self.f(st + step_horizon/2*k2, con)
            k4 = self.f(st + step_horizon * k3, con)
            st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, st_next - st_next_RK4)


        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': cost_fn,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        self.lbx = ca.DM.zeros((self.n_states*(N+1) + self.n_controls*N, 1))
        self.ubx = ca.DM.zeros((self.n_states*(N+1) + self.n_controls*N, 1))

        self.lbx[0: self.n_states*(N+1): self.n_states] = -ca.inf     # X lower bound
        self.lbx[1: self.n_states*(N+1): self.n_states] = -ca.inf     # Y lower bound
        self.lbx[2: self.n_states*(N+1): self.n_states] = -ca.inf     # theta lower bound

        self.ubx[0: self.n_states*(N+1): self.n_states] = ca.inf      # X upper bound
        self.ubx[1: self.n_states*(N+1): self.n_states] = ca.inf      # Y upper bound
        self.ubx[2: self.n_states*(N+1): self.n_states] = ca.inf      # theta upper bound
        
        self.state_init = ca.DM([self.x_init, self.y_init, self.theta_init])        # initial state
        self.state_target = ca.DM([self.x_target, self.y_target, self.theta_target])  # target state

        self.u0 = ca.DM.zeros((self.n_controls, N))  # initial control
        self.X0 = ca.repmat(self.state_init, 1, N+1)         # initial state full


        self.cat_states = DM2Arr(self.X0)
        self.cat_controls = DM2Arr(self.u0[:, 0])

        
        self.t_test = 0

        self.t = 0
    ####################################

    def mpc_callback(self):
        speed_pub = Float32MultiArray()
        if ((ca.norm_2(self.state_init - self.state_target)) > 1e-2 and self.goal == False):
            self.lbx[self.n_states*(N+1):] = -self.v_max                  # v lower bound for all V
            self.ubx[self.n_states*(N+1):] = self.v_max                  # v upper bound for all V
            self.args = {
                'lbg': ca.DM.zeros((self.n_states*(N+1), 1)),  # constraints lower bound
                'ubg': ca.DM.zeros((self.n_states*(N+1), 1)),  # constraints upper bound
                'lbx': self.lbx,
                'ubx': self.ubx
            }
            self.args['p'] = ca.vertcat(
                self.state_init,    # current state
                self.state_target   # target state
            )
            # optimization variable current state
            self.args['x0'] = ca.vertcat(
                ca.reshape(self.X0, self.n_states*(N+1), 1),
                ca.reshape(self.u0, self.n_controls*N, 1)
            )

            sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )

            u = ca.reshape(sol['x'][self.n_states * (N + 1):], self.n_controls, N)
            self.X0 = ca.reshape(sol['x'][: self.n_states * (N+1)], self.n_states, N+1)


            self.V_pub = u[:,0]
            print(self.V_pub)
            for i in range(4):
                if (self.V_pub[i] < 0.2 and self.V_pub[i] > -0.2):
                    self.V_pub[i] = 0
            
            #######################################
            self.cat_states = np.dstack((
                self.cat_states,
                DM2Arr(self.X0)
            ))

            self.cat_controls = np.vstack((
                self.cat_controls,
                DM2Arr(u[:, 0])
            ))
            ###### close loop 
            self.state_init, self.u0 = self.shift_timestep(u)


            self.X0 = ca.horzcat(
                self.X0[:, 1:],
                ca.reshape(self.X0[:, -1], -1, 1)
            )
            ###############
            self.t_test = self.t_test + 0.01
            self.t = self.t + 0.1
            self.v_max = self.v_max + 20 * 0.1
            if self.v_max >= V_limit : self.v_max = V_limit
        else :
            self.t = 0
            self.V_pub = [0,0,0,0]
            self.goal = True
            print("goal!!!")
        speed_pub.data =  [(float)(self.V_pub[0]),(float) (self.V_pub[1]),(float) (self.V_pub[2]),(float)(self.V_pub[3])]
        self.publisher_.publish(speed_pub)


    def shift_timestep(self,u):
        f_value = self.f(self.state_init, u[:, 0])
        next_state = ca.DM.full(self.state_init + (step_horizon * f_value))
        self.u0 = ca.horzcat(
            u[:, 1:],
            ca.reshape(u[:, -1], -1, 1)
        )

        return next_state, self.u0


def main(args=None):
    rclpy.init(args=args)

    mpc_node_test = mpc_class()

    rclpy.spin(mpc_node_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc_node_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()