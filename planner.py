import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Trajectory types
PARABOLA_TRAJECTORY=0; SIGMOID_TRAJECTORY=1



class planner:
    def __init__(self, type_):
        self.type=type_
        # Hardcoded trajectory selection - change this to switch between trajectories
        self.trajectory_type = PARABOLA_TRAJECTORY  # Change to SIGMOID_TRAJECTORY for sigmoid

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        
        if self.trajectory_type == PARABOLA_TRAJECTORY:
            return self.parabola_trajectory()
        elif self.trajectory_type == SIGMOID_TRAJECTORY:
            return self.sigmoid_trajectory()
        else:
            # Default to parabola if unknown trajectory type
            return self.parabola_trajectory()
    
    def parabola_trajectory(self):
        """
        Parabola trajectory: y = x^2 for x ∈ [0.0, 1.5]
        Returns a list of trajectory points: [[x1,y1], ..., [xn,yn]]
        """
        
        # Generate x values from 0.0 to 1.5 with appropriate step size
        x = np.linspace(0.0, 1.5, 50)  # 50 points for smooth trajectory
        y = x**2  # y = x^2
        
        # Convert to list of [x, y] points
        trajectory = [[float(x[i]), float(y[i])] for i in range(len(x))]
        
        return trajectory
    
    def sigmoid_trajectory(self):
        """
        Sigmoid trajectory: σ(x) = 2/(1 + e^(-2x)) - 1 for x ∈ [0.0, 2.5]
        Returns a list of trajectory points: [[x1,y1], ..., [xn,yn]]
        """
        import numpy as np
        
        # Generate x values from 0.0 to 2.5 with appropriate step size
        x = np.linspace(0.0, 2.5, 50)  # 50 points for smooth trajectory
        y = 2.0 / (1.0 + np.exp(-2.0 * x)) - 1.0  # σ(x) = 2/(1 + e^(-2x)) - 1
        
        # Convert to list of [x, y] points
        trajectory = [[float(x[i]), float(y[i])] for i in range(len(x))]
        
        return trajectory

