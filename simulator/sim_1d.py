import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from scipy.optimize import minimize
import time


def run_sim(options, MPC):
    start = time.process_time()
    # Simulator Options
    FULL_RECALCULATE = options['FULL_RECALCULATE']
    FIG_SIZE = options['FIG_SIZE']  # [Width, Height]

    mpc = MPC()

    # initialize the control input
    num_inputs = 2
    u = np.zeros(mpc.horizon*num_inputs)

    bounds = []
    # Set constraint for inputs bounded optimization.
    for i in range(mpc.horizon):
        bounds += [[-1, 1]]
        bounds += [[-0.0, 0.0]]

    ref = mpc.reference

    state_i = np.array([[1, 0, 0, 0]])
    u_i = np.array([[0, 0]])
    sim_total = 250
    predict_info = [state_i]

    for i in range(1, sim_total+1):
        # Reuse old inputs as starting point to decrease run time.
        u = np.delete(u, 0)
        u = np.delete(u, 0)
        u = np.append(u, u[-2])
        u = np.append(u, u[-2])
        if (FULL_RECALCULATE):
            u = np.zeros(mpc.horizon*num_inputs)
        start_time = time.time()

        # Non-linear optimization.
        u_solution = minimize(mpc.cost_func, u, (state_i[-1], ref),
                              method='SLSQP',
                              bounds=bounds,
                              tol=1e-5)
        print('Step ' + str(i) + ' of ' + str(sim_total) + '   Time ' + str(round(time.time() - start_time,5)))
        u = u_solution.x
        # Predict the future states
        predicted_next = mpc.system_model(state_i[-1], mpc.dt, u[0], u[1])
        predicted_state = np.array([predicted_next])
        for j in range(1, mpc.horizon):
            predicted = mpc.system_model(predicted_state[-1], mpc.dt, u[2 * j], u[2 * j + 1])
            predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
        predict_info += [predicted_state]
        # Assume our model is perfect, so the next_real_state=predicted_next
        state_i = np.append(state_i, np.array([predicted_next]), axis=0)
        u_i = np.append(u_i, np.array([(u[0], u[1])]), axis=0)


    ###################
    # DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(8,8)

    # plot settings.
    ax = fig.add_subplot(gs[:8, :8])


    ax.set_ylim(-3, 17)
    plt.xticks(np.arange(0,90, step=2))
    plt.xlim(0, 20)
    plt.yticks([])
    plt.title('MPC 1D')

    # Main plot info.
    car_width = 1.0
    car = mpatches.Rectangle((0, 0), car_width, 2.5, fc='m', fill=True)
    goal = mpatches.Rectangle((0, 0), car_width, 2.5, fc='b', ls='dashdot', fill=False)

    ax.add_patch(car)
    ax.add_patch(goal)
    predict, = ax.plot([], [], 'r--', linewidth=1)

    # Car steering and gas position.
    loc = [6, 14]
    wheel = mpatches.Circle((loc[0]-3, loc[1]), 2.2, fill=False, fc='r')
    ax.add_patch(wheel)
    wheel_1, = ax.plot([], [], 'b', linewidth=3)
    wheel_2, = ax.plot([], [], 'b', linewidth=3)
    wheel_3, = ax.plot([], [], 'b', linewidth=3)
    gas, = ax.plot([], [], 'k', linewidth=20)
    brake, = ax.plot([], [], 'k', linewidth=20)
    gas_text = ax.text(loc[0], loc[1]-3, 'Forward', fontsize=13,
                       horizontalalignment='center')
    brake_text = ax.text(loc[0]+3, loc[1]-3, 'Reverse', fontsize=13,
                         horizontalalignment='center')


    # Speed Indicator
    speed_text = ax.text(loc[0]+7, loc[1], '0', fontsize=13)
    speed_units_text = ax.text(loc[0]+8.5, loc[1], 'km/h (Speed)', fontsize=13)

    # Shift xy, centered on the rear left corner of car.
    def car_pos(x, y, psi):
        x_new = x - np.sin(psi)*(car_width/2)
        y_new = y + np.cos(psi)*(car_width/2)
        return [x_new, y_new]

    def wheel_steering(wheel_angle):
        wheel_1.set_data([loc[0]-3, loc[0]-3+np.cos(wheel_angle)*2],
                         [loc[1], loc[1]+np.sin(wheel_angle)*2])
        wheel_2.set_data([loc[0]-3, loc[0]-3-np.cos(wheel_angle)*2],
                         [loc[1], loc[1]-np.sin(wheel_angle)*2])
        wheel_3.set_data([loc[0]-3, loc[0]-3+np.sin(wheel_angle)*2],
                         [loc[1], loc[1]-np.cos(wheel_angle)*2])
        brake_text.set_x(loc[0]+3)
        gas_text.set_x(loc[0])
        wheel.center = loc[0]-3, loc[1]
        speed_text.set_x(loc[0] + 7)
        speed_units_text.set_x(loc[0] + 8.5)

    def plot_update(num):
        # Car.
        car.set_xy(car_pos(state_i[num,0], state_i[num,1], state_i[num,2]))
        car.angle = np.rad2deg(state_i[num,2])-90
        # Car wheels
        wheel_steering(u_i[num, 1]*2)
        gas.set_data([loc[0], loc[0]],
                        [loc[1]-2, loc[1]-2+max(0, u_i[num, 0]/1*4)])
        brake.set_data([loc[0]+3, loc[0]+3],
                        [loc[1]-2, loc[1]-2+max(0, -u_i[num, 0]/1*4)])

        speed = state_i[num, 3]*3.6
        speed_text.set_text(str(round(speed,1)))
        if speed > 30.1:
            speed_text.set_color('r')
        else:
            speed_text.set_color('k')

        goal.set_xy(car_pos(ref[0], ref[1], ref[2]))
        goal.angle = np.rad2deg(ref[2])-90

        predict.set_data(predict_info[num][:,0], predict_info[num][:,1])

        if (state_i[num,0] > 5):
            plt.xlim(state_i[num,0]-5, state_i[num,0]+15)
            loc[0] = state_i[num,0]+1

        return car

    print("Compute Time: ", round(time.process_time() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, plot_update, frames=range(1, len(state_i)),
                                      interval=100, repeat=True, blit=False)
    # car_ani.save('mpc_1d_video.mp4')

    plt.show()
