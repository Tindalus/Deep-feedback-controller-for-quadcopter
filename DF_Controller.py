import numpy as np


def Controller(State:list, DesiredState:list, Parameters:list, Accelerations:list,Ts:float, DesiredAccelerations:list = [0,0,0,0,0,0]):
    """
    This function generates control signal in the closed loop system

    Args:
        State (list (12)): Current state of the system.
        DesiredState (list (12)): List of desired trajectory coordinates.
        Parameters (list (12)): List of controller paremeters.
        Accelerations (list (6)): List of accelerations on the current step.
        Ts (float): Numerical scheme step.
        DesiredAccelerations (list (6), optional): list of desired accelerations. Rarely used, default [0,0,0,0,0,0].
    Returns:
        list: list of 6 input signals in order: [u1,u2,u3,tau_psi,tau_phi,tau_theta]
    """
    #reshaping DesiredState
    new_order_indices = np.array([0, 1, 2, 3, 4, 5, 8, 9, 6, 7, 10, 11])
    DesiredState = DesiredState[new_order_indices]

    #Forming error vector
    Errors = np.array(DesiredState)-np.array(State)

    #Additional acceleration error vector
    AccelErrors = np.array(DesiredAccelerations)-np.array(Accelerations)

    K_Z = 100
    u3 = K_Z*(AccelErrors[0]*Ts + Parameters[0]*Errors[1]*Ts + Parameters[1]*Errors[0]*Ts + Accelerations[0]*Ts)

    K_X = 150
    u1 = K_X*(AccelErrors[1]*Ts + Parameters[2]*Errors[3]*Ts + Parameters[3]*Errors[2]*Ts + Accelerations[1]*Ts)

    K_Y = 150
    u2 = K_Y*(AccelErrors[2]*Ts + Parameters[4]*Errors[5]*Ts + Parameters[5]*Errors[4]*Ts + Accelerations[2]*Ts)

    K_Psi = 500
    tau_psi = K_Psi*(AccelErrors[5]*Ts + Parameters[10]*Errors[10]*Ts + Parameters[11]*Errors[11]*Ts + Accelerations[5]*Ts)
    # print(AccelErrors[5]*Ts + Parameters[10]*Errors[6]*Ts)
    K_Phi = 150
    tau_phi = K_Phi*(AccelErrors[3]*Ts + Parameters[6]*Errors[7]*Ts + Parameters[7]*Errors[6]*Ts + Accelerations[3]*Ts)
    
    K_T = 150
    tau_theta = K_T*(AccelErrors[4]*Ts + Parameters[8]*Errors[9]*Ts + Parameters[9]*Errors[8]*Ts + Accelerations[4]*Ts)

    #u3 is respozible for z coord control, u1 for x and u2 for y
    return [u1,u2,u3,tau_theta,tau_phi,tau_psi]
