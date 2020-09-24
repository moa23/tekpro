import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time



N = 1
initial_conditions = np.array(np.mat('0.01;0.01;0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

# Create unicycle pose controller
unicycle_pose_controller = create_hybrid_unicycle_pose_controller()

# Create barrier certificates to avoid collision
uni_barrier_cert = create_unicycle_barrier_certificate()
x= r.get_poses()

arrayExtra = np.array([2.5, 0, 0])
arrayExtra.shape = (3,1)
goal_points = arrayExtra




CM = np.random.rand(N,3) # Random Colors
goal_marker_size_m = 0.2
robot_marker_size_m = 0.15
marker_size_goal = determine_marker_size(r,goal_marker_size_m)
marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r,0.1)
line_width = 3

goal_caption = ['G{0}'.format(ii) for ii in range(goal_points.shape[1])]
#Arrow for desired orientation
goal_orientation_arrows = [r.axes.arrow(goal_points[0,ii], goal_points[1,ii], goal_marker_size_m*np.cos(goal_points[2,ii]), goal_marker_size_m*np.sin(goal_points[2,ii]), width = 0.02, length_includes_head=True, color = CM[ii,:], zorder=-2)
for ii in range(goal_points.shape[1])]
#Plot text for caption
goal_points_text = [r.axes.text(goal_points[0,ii], goal_points[1,ii], goal_caption[ii], fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-3)
for ii in range(goal_points.shape[1])]
goal_markers = [r.axes.scatter(goal_points[0,ii], goal_points[1,ii], s=marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-3)
for ii in range(goal_points.shape[1])]
robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width) 
for ii in range(goal_points.shape[1])]


position_controller = create_si_position_controller();

si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()




countMax        = 150           # integer number of steps
laneRadius= float(0.4)

   

circle = plt.Circle( (-0.2, 0),  laneRadius, color='b',fill = False, linewidth = line_width)
circle2 = plt.Circle( (0.55, 0),  laneRadius, color='r',fill = False, linewidth = line_width)

r.axes.add_artist(circle)
r.axes.add_artist(circle2)




maxLinearSpeed  = float(0.2) # extracted from line 47 of robotarium_abc.py
maxAngularSpeed = float(3.9) # extracted from line 48 of robotarium_abc.py
linearSpeed     = float(0.15)
if ( linearSpeed * linearSpeed > maxLinearSpeed * maxLinearSpeed ):
    linearSpeed = maxLinearSpeed
kProportional   = float(1)
kDerivative     = float(0.3)
kCurvature      = linearSpeed / laneRadius 
smallRadius     = laneRadius / float(10)

r.step()

xyz= r.get_poses()





r.step()
count=0


while((count < countMax)):
    xyz = r.get_poses()
    for j in range(goal_points.shape[1]):
        goal_markers[j].set_sizes([determine_marker_size(r, goal_marker_size_m)])

    # Create unicycle control inputs -- is needed for plotting
    dxu = unicycle_pose_controller(x, goal_points)

    # Create safe control inputs (i.e., no collisions)
    dxu = uni_barrier_cert(dxu, x)
    # -------------- up to here

    # increment simulation step count
    count = count + 1

    # calculate present location and orientation
    xRob     = float(x.item(0))
    yRob     = float(x.item(1))
    psiRob   = float(x.item(2))
    rRob     = np.sqrt( float( xRob * xRob + yRob * yRob )  )

    #    if the robot is too close to the origin
    if ( rRob <= smallRadius ):
        thetaRob = psiRob   
    #    otherwise
    else:
        thetaRob = np.arctan2( yRob , xRob )
    # and the orientation error is:
    dpsi     = psiRob - thetaRob - np.pi / float (2)
    # and the lateral error is
    dy       = laneRadius - rRob

    r.step()




r.call_at_scripts_end()
