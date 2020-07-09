#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt

from .context import Planners

def main():
    np.set_printoptions(precision=2)

    c = Planners.VFHP.VConst()
    c.DIST_FCN = 'GAUSS'
    c.E = 3.2
    c.T_LO = 1000
    c.T_HI = 1800

    robot = Planners.VFHP.VFHPModel(c)
    print "Obstacle grid"
    print robot.obstacle_grid, "\n"
    print "Active Window angles"
    print robot.active_window[:,:,Planners.VFHP.BETA], "\n"
    print "Active Window squared distances"
    print robot.active_window[:,:,Planners.VFHP.DIST2], "\n"
    print "Active Window a-bd^2 constants"
    print robot.active_window[:,:,Planners.VFHP.ABDIST], "\n"
    print "Max distance squared: %f" % c.D_max2


    print("Updating the obstacle grid and robot position")

    robot.update_position(1.3+0.15, 1.5, math.pi*6/6)

    #robot.obstacle_grid[1,6] = 1
    #robot.obstacle_grid[1,5] = 2
    #robot.obstacle_grid[1,4] = 2
    #robot.obstacle_grid[1,3] = 5
    #robot.obstacle_grid[2,2] = 3
    #robot.obstacle_grid[3,2] = 3
    #robot.obstacle_grid[4,2] = 3
#
#    robot.obstacle_grid[9,2] = 4
#    robot.obstacle_grid[9,3] = 5
#    robot.obstacle_grid[9,4] = 6
#    robot.obstacle_grid[9,5] = 5
#    robot.obstacle_grid[9,6] = 4

    robot.obstacle_grid[27,30] = c.C_MAX
    robot.obstacle_grid[27,31] = c.C_MAX
    robot.obstacle_grid[27,29] = c.C_MAX
    robot.obstacle_grid[28,30] = c.C_MAX
    robot.obstacle_grid[26,30] = c.C_MAX

    robot.obstacle_grid[30,37] = c.C_MAX
    robot.obstacle_grid[31,37] = c.C_MAX
    robot.obstacle_grid[29,37] = c.C_MAX
    robot.obstacle_grid[30,38] = c.C_MAX
    robot.obstacle_grid[30,36] = c.C_MAX

    robot.obstacle_grid[41,30] = c.C_MAX
    robot.obstacle_grid[40,30] = c.C_MAX
    robot.obstacle_grid[42,30] = c.C_MAX
    robot.obstacle_grid[41,31] = c.C_MAX
    robot.obstacle_grid[41,29] = c.C_MAX

    print "i , j , k "
    print robot.i_0, robot.j_0, robot.k_0
    print robot._active_grid(), "\n"

    print "Simulating a set of sensor readings"
    pseudo_readings = np.float_([[0.2, np.radians(x)] for x in range(0,45,2)])
    #pseudo_readings = np.float_([[0.3,
    robot.update_obstacle_density(pseudo_readings)


    print "Updating the active window"
    robot.update_active_window()
    print robot.active_window[:, :, Planners.VFHP.MAG], "\n"

    print "Updating polar histogram"
    robot.update_polar_histogram()
    print robot.polar_hist, "\n"

    print "Updating binary histogram"
    robot.update_bin_polar_histogram()
    print robot.bin_polar_hist, "\n"

    print "Updating masked polar histogram"
    robot.update_masked_polar_hist(c.R_ROB*50,c.R_ROB*5)
    print robot.masked_polar_hist, "\n"

    print "Looking for valleys"
    i = robot.find_valleys()
    print robot.valleys, "\n"

    print "Select new direction"
    robot.prev_dir = math.pi/2
    print robot.calculate_steering_dir(i), "\n"

    print "Setting speed to (MAX = {:.2f})".format(c.V_MAX)
    print robot.calculate_speed(), "\n"


#    print "Updating filtered histogram"
#    robot.update_filtered_polar_histogram()
#    print robot.filt_polar_hist, "\n"
#
#    print "Looking for valleys"
#    robot.find_valleys()
#    print robot.valleys, "\n"
#
#    try:
#        print "Setting steer direction"
#        cita = robot.calculate_steering_dir()
#        print cita, "\n"
#    except:
#        pass

    ### Figuras y graficos ###
#
#    plt.figure(2)
#    plt.plot(i, robot.filt_polar_hist ) #, 0.1, 0, color='b')
#    plt.title("Histograma polar filtrado")

    plt.figure(1)
    plt.pcolor(robot._active_grid().T, alpha=0.75, edgecolors='k',vmin=0,vmax=20)
    plt.xlabel("X")
    plt.ylabel("Y", rotation='horizontal')

    plt.figure(2)
    x = np.degrees([c.ALPHA*x for x in range(len(robot.polar_hist))])
    i = [a for a in range(len(robot.polar_hist))]
    plt.bar(x, robot.polar_hist, math.degrees(c.ALPHA), 0, color='r')
    plt.title("Histograma polar")

    plt.figure(3)
    x = np.degrees([c.ALPHA*x for x in range(len(robot.bin_polar_hist))])
    i = [a for a in range(len(robot.bin_polar_hist))]
    plt.bar(x, robot.bin_polar_hist, math.degrees(c.ALPHA), 0, color='b')
    plt.title("Histograma polar binario")

    plt.figure(4)
    x = np.degrees([c.ALPHA*x for x in range(len(robot.masked_polar_hist))])
    i = [a for a in range(len(robot.masked_polar_hist))]
    plt.bar(x, robot.masked_polar_hist, math.degrees(c.ALPHA), 0, color='g')
    plt.title("Histograma polar mascarado")

    plt.show()

if __name__ == "__main__":
    main()
