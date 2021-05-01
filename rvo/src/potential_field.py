import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


class PotentialField():
    # A 2D potential field
    def __init__(self, goal=None, obstacles=[],
                       k_goal=0.2, goal_star=1, k_obstacle=1, obstacles_min=0.5, obstacles_star=2):
        # Store data
        # goal (x, y)
        self.goal = goal
        # obstacles [(x1, y1, r1), (x2, y2, r2)]
        self.obstacles = obstacles

        # Parameters for computing potential field
        # attractive
        self.k_goal = k_goal
        self.goal_star = goal_star
        # repulsive
        self.k_obstacle = k_obstacle
        self.obstacles_min = obstacles_min
        self.obstacles_star = obstacles_star


    def clear(self):
        self.goal = None
        self.obstacles = []


    def set_goal(self, goal):
        self.goal = goal


    def set_parameters(self, k_goal=0.2, goal_star=1, k_obstacle=1, obstacles_min=0.5, obstacles_star=2):
        # Parameters for computing potential field
        # attractive
        self.k_goal = k_goal
        self.goal_star = goal_star
        # repulsive
        self.k_obstacle = k_obstacle
        self.obstacles_min = obstacles_min
        self.obstacles_star = obstacles_star


    def add_obstacles(self, obstacles):
        # Determine the layer of obstacles
        def compute_depth(list_var):
            for i in list_var:
                if not isinstance(i, list) and not isinstance(i, tuple):
                    return 1
                else:
                    return compute_depth(i) + 1
        layer = compute_depth(obstacles)
        print(layer)

        # Single obstacle
        if layer == 1:
            self.obstacles.append(obstacles)
        # Multiple obstacles
        elif layer == 2:
            for obstacle in obstacles:
                self.obstacles.append(obstacle)
        else:
            print("Obstacle type incorrect.")


    def compute_attractive_potential(self, pos):
        # Attractive potential
        pos = np.array(pos)
        goal = np.array(self.goal)

        # distance to goal
        dis_to_goal = np.linalg.norm(pos - goal)

        # quadratic
        if dis_to_goal <= self.goal_star:
            z_attr = self.k_goal * 0.5 * dis_to_goal**2
        # linear
        else:
            z_attr = self.k_goal *(-0.5 * self.goal_star**2 + \
                                   self.goal_star * dis_to_goal)
        
        return z_attr


    def compute_repulsive_potential(self, pos):
        # Repulsive force
        pos = np.array(pos)

        z_repul = 0.0
        # For each obstacle
        for obstacle in self.obstacles:
            obstacle_pos = np.array(obstacle[0:2])
            obstacle_r = obstacle[2]
        
            # distance to all obstacles
            dis_to_obs = np.linalg.norm(pos - obstacle_pos) - obstacle_r

            # when it is too close to obstacle
            if dis_to_obs <= self.obstacles_min:
                z = self.k_obstacle * 0.5 * (1/self.obstacles_min - 1/self.obstacles_star)**2
            # when it is in range star
            elif dis_to_obs <= self.obstacles_star:
                z = self.k_obstacle * 0.5 * (1/dis_to_obs - 1/self.obstacles_star)**2
            # far from obstacle
            else:
                z = 0

            z_repul += z
        
        return z_repul


    def compute_potential(self, pos):
        attractive_potential = self.compute_attractive_potential(pos)
        repulsive_potential = self.compute_repulsive_potential(pos)

        return attractive_potential + repulsive_potential


    def compute_attractive_force(self, pos):
        # Attractive force
        pos = np.array(pos)
        goal = np.array(self.goal)

        # distance to goal
        dis_to_goal = np.linalg.norm(pos - goal)

        # quadratic
        if dis_to_goal <= self.goal_star:
            F_attr = self.k_goal * (pos - goal)
        # linear
        else:
            F_attr = self.k_goal * self.goal_star * (pos - goal) / dis_to_goal

        return - F_attr


    def compute_repulsive_force(self, pos):
        # Repulsive force
        pos = np.array(pos)

        F_repl = np.array([0.0, 0.0])
        # For each obstacle
        for obstacle in self.obstacles:
            obstacle_pos = np.array(obstacle[0:2])
            obstacle_r = obstacle[2]
        
            # distance to all obstacles
            dis_to_obs = np.linalg.norm(pos - obstacle_pos) - obstacle_r
            
            # when it is too close to obstacle
            if dis_to_obs <= self.obstacles_min:
                F = np.array([0.0, 0.0])
            # considered only when it is in range star
            elif dis_to_obs <= self.obstacles_star:
                F = self.k_obstacle * (1/self.obstacles_star - 1/dis_to_obs) * \
                    1/dis_to_obs**2 * (pos - obstacle_pos)/dis_to_obs
            else:
                F = np.array([0.0, 0.0])

            F_repl += F

        return - F_repl


    def compute_force(self, pos):
        F_attr = self.compute_attractive_force(pos)
        F_repl = self.compute_repulsive_force(pos)

        return F_attr + F_repl

    
    def visualize_potential(self, arange=[-4.0, 4.0, -4.0, 4.0], step=0.25):
        # Compute potential field in range
        X = np.arange(arange[0], arange[1], step)
        Y = np.arange(arange[2], arange[3], step)
        X, Y = np.meshgrid(X, Y)
        Z = np.zeros_like(X)
        for i in range(len(X)):
            for j in range(len(X[0])):
                Z[i][j] = pf.compute_potential((X[i][j], Y[i][j]))

        # Plot the surface
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                            linewidth=0, antialiased=False)
        # Add a color bar which maps values to colors
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.show()

    
    def visualize_vector_field(self, arange=[-4.0, 4.0, -4.0, 4.0], step=0.25):
        # Compute potential field in range
        X = np.arange(arange[0], arange[1], step)
        Y = np.arange(arange[2], arange[3], step)
        X, Y = np.meshgrid(X, Y)
        
        U = np.zeros_like(X)
        V = np.zeros_like(X)
        for i in range(len(X)):
            for j in range(len(X[0])):
                F = pf.compute_force((X[i][j], Y[i][j]))
                
                # Change the value just to make the graph looks better
                if abs(F[0]) > 2 or abs(F[1]) > 2:
                    F[0] *= 0.1
                    F[1] *= 0.1

                U[i][j] = F[0]
                V[i][j] = F[1]

        # Plot the surface
        fig, ax = plt.subplots()
        Q = ax.quiver(X, Y, U, V, scale=10, units='width')

        plt.show()


if __name__ == "__main__":
    # Example
    pf = PotentialField()
    pf.set_goal((3, 3))
    pf.add_obstacles((1, 1, 1))
    # pf.add_obstacles([(2, 0, 0.5), (-1, 0, 0.5)])

    # compute potential value
    potential_at_zero = pf.compute_potential((0, 0))
    print(potential_at_zero)

    # compute force
    force_at_zero = pf.compute_force((0, 0))
    print(force_at_zero)

    # next step
    descent_rate = 1.0
    next_move = descent_rate * force_at_zero
    delta_x = next_move[0]
    delta_y = next_move[1]
    print(delta_x, delta_y)

    # Visualization
    pf.visualize_potential()
    pf.visualize_vector_field()
