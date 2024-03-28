from copy import copy
import numpy as np
import sys
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Synchronization, ControlInterface, DurationDiscretization
# from ruckig import Reflexxes


def walk_through_trajectory(otg, inp):
    out_list = []
    out_pos =[]
    out_time= []
    out = OutputParameter(inp.degrees_of_freedom)

    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        inp.current_position = out.new_position
        inp.current_velocity = out.new_velocity
        inp.current_acceleration = out.new_acceleration

        out_list.append(copy(out))
        out_pos.append((out.new_position))
        out_time.append(out.calculation_duration)

    return out_list,out_pos,out_time


# if __name__ == '__main__':
def ruckig_pos(c_pos,t_pos,max_vel,max_acc,max_jerk,step_time):
    # c_pos =[0,0,0,0,0,0]
    # t_pos =[1,1,1,0,0,0]
    # max_vel= [100,100,100,100,100,100]
    # max_acc= [50,50,50,50,50,50]
    # max_jerk= [10,10,10,10,10,10]
    # step_time= 0.01

    inp = InputParameter(6)
    # inp.control_interface = ControlInterface.Velocity
    # inp.synchronization = Synchronization.No
    # inp.duration_discretization = DurationDiscretization.Discrete

    # inp.per_dof_control_interface = [ControlInterface.Position, ControlInterface.Velocity, ControlInterface.Position]
    # inp.per_dof_synchronization = [Synchronization.Phase, Synchronization.Time, Synchronization.Phase]

    c_pos = list(map(float, c_pos.split()))
    inp.current_position = c_pos
    inp.current_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    inp.current_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    t_pos = list(map(float, t_pos.split()))
    inp.target_position = t_pos
    inp.target_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    inp.target_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    max_vel = list(map(float, max_vel.split()))
    inp.max_velocity = max_vel

    max_acc = list(map(float, max_acc.split()))
    inp.max_acceleration = max_acc

    max_jerk = list(map(float, max_jerk.split()))
    inp.max_jerk = max_jerk

    # inp.minimum_duration = 5.0

    step_time = float(step_time)
    # otg = Reflexxes(inp.degrees_of_freedom, 0.005)
    otg = Ruckig(inp.degrees_of_freedom, step_time)

    out_list,out_pos,out_time = walk_through_trajectory(otg, inp)
    return out_pos


# Đảm bảo chương trình chỉ được chạy khi được gọi từ dòng lệnh
if __name__ == "__main__":
    # Nhận đầu vào từ dòng lệnh
    c_pos_m = sys.argv[1]
    t_pos_m = sys.argv[2]
    max_vel_m = sys.argv[3]
    max_acc_m = sys.argv[4]
    max_jerk_m = sys.argv[5]
    step_time_m = sys.argv[6]

    # Gọi hàm ruckig_pos với đầu vào từ dòng lệnh
    result = ruckig_pos(c_pos_m, t_pos_m, max_vel_m, max_acc_m, max_jerk_m, step_time_m)

    # In kết quả
    print(result)