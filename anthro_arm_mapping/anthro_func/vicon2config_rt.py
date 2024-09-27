import numpy as np


def frame2calibrate_rt(data_array_cali, robot):
    '''
    Return the transform from marker shoulder frame to calibrated shoulder frame,
    the transform from marker wrist frame to calibrated wrist frame and the length scaling factor
    '''
    shoulder_back = data_array_cali[:3]
    shoulder_front = data_array_cali[3:6]
    shoulder_up = data_array_cali[6:9]
    elbow_lateral = data_array_cali[9:12]
    elbow_medial = data_array_cali[12:15]
    wrist_front_thumb = data_array_cali[15:18]
    wrist_back_pinky = data_array_cali[18:21]
    hand = data_array_cali[21:24]
    ''' Constant transform shoulder to shoulder reference frame'''
    # Shoulder reference frame
    p_shoulder = (shoulder_front + shoulder_back) / 2
    p_wrist = (wrist_front_thumb + wrist_back_pinky) / 2
    z_shoulder_ref = p_wrist - p_shoulder
    z_shoulder_ref[2] = 0.0
    z_shoulder_ref = z_shoulder_ref / np.linalg.norm(z_shoulder_ref)
    y_shoulder_ref = np.array([0.0, 0.0, 1.0])
    x_shoulder_ref = np.cross(y_shoulder_ref, z_shoulder_ref)
    x_shoulder_ref = x_shoulder_ref / np.linalg.norm(x_shoulder_ref)
    R_bsr = np.array(
        [x_shoulder_ref, y_shoulder_ref, z_shoulder_ref]
    ).T
    # Orientation of shoulder
    z_orient_shoulder = (shoulder_front - shoulder_back) / np.linalg.norm(shoulder_front - shoulder_back)
    x_orient_shoulder = np.cross((shoulder_up - shoulder_back), z_orient_shoulder)
    x_orient_shoulder = x_orient_shoulder / np.linalg.norm(x_orient_shoulder)
    y_orient_shoulder = np.cross(z_orient_shoulder, x_orient_shoulder)
    y_orient_shoulder = y_orient_shoulder / np.linalg.norm(y_orient_shoulder)

    # Transpose of orientation matrix of shoulder
    R_bs_T = np.array(
        [x_orient_shoulder, y_orient_shoulder, z_orient_shoulder]
    )
    # The constant transform shoulder to shoulder reference frame
    R_ssr_const = R_bs_T @ R_bsr
    ''' Constant transform wrist to wrist reference frame'''
    p_elbow = (elbow_lateral + elbow_medial) / 2
    # Wrist reference frame
    x_wrist_ref = (p_wrist - p_elbow) / np.linalg.norm(p_wrist - p_elbow)
    y_wrist_ref = np.cross(p_shoulder - p_wrist, x_wrist_ref)
    y_wrist_ref = y_wrist_ref / np.linalg.norm(y_wrist_ref)
    z_wrist_ref = np.cross(x_wrist_ref, y_wrist_ref)
    R_bwr = np.array(
        [x_wrist_ref, y_wrist_ref, z_wrist_ref]
    ).T
    # Orientation of wrist
    y_orient_wrist = (wrist_back_pinky - wrist_front_thumb) / np.linalg.norm(
        wrist_back_pinky - wrist_front_thumb)
    z_orient_wrist = np.cross(hand - wrist_front_thumb, y_orient_wrist)
    z_orient_wrist = z_orient_wrist / np.linalg.norm(z_orient_wrist)
    x_orient_wrist = np.cross(y_orient_wrist, z_orient_wrist)
    # Transpose of orientation matrix of wrist
    R_bw_T = np.array(
        [x_orient_wrist, y_orient_wrist, z_orient_wrist]
    )
    # The constant transform wrist to wrist reference frame
    R_wwr_const = R_bw_T @ R_bwr
    ''' Length scaling '''
    length_human = np.linalg.norm(p_elbow - p_shoulder) + np.linalg.norm(p_wrist - p_elbow)
    # length_scaler = robot.robot_length / length_human
    robot_lower = 0.3
    human_lower = 0.15
    # human_lower = np.linalg.norm(p_wrist - p_shoulder)
    length_scaler = (robot.robot_length - robot_lower) / (length_human - human_lower)
    return R_ssr_const, R_wwr_const, length_scaler

def vicon2constraint_rt(data_array, R_ssr_const, R_wwr_const, length_scaler):
    '''
    Convert real-time Vicon data to real-time constraint data (wrist pose and nominal vector of arm plane)
    '''
    ''' Read data '''
    shoulder_back = data_array[:3]
    shoulder_front = data_array[3:6]
    shoulder_up = data_array[6:9]
    elbow_lateral = data_array[9:12]
    elbow_medial = data_array[12:15]
    wrist_front_thumb = data_array[15:18]
    wrist_back_pinky = data_array[18:21]
    hand = data_array[21:24]
    ''' Shoulder '''
    # Position of shoulder
    p_shoulder = (shoulder_front + shoulder_back) / 2
    # Orientation of shoulder
    z_orient_shoulder = (shoulder_front - shoulder_back) / np.linalg.norm(shoulder_front - shoulder_back)
    x_orient_shoulder = np.cross((shoulder_up - shoulder_back), z_orient_shoulder)
    x_orient_shoulder = x_orient_shoulder / np.linalg.norm(x_orient_shoulder)
    y_orient_shoulder = np.cross(z_orient_shoulder, x_orient_shoulder)
    y_orient_shoulder = y_orient_shoulder / np.linalg.norm(y_orient_shoulder)
    # Orientation matrix of shoulder
    R_bs = np.array(
        [x_orient_shoulder, y_orient_shoulder, z_orient_shoulder]
    ).T
    R_bs = R_bs @ R_ssr_const
    R_bs_T = R_bs.T
    ''' Elbow '''
    # Position of elbow
    p_elbow = (elbow_lateral + elbow_medial) / 2
    ''' Wrist '''
    # Position of wrist
    p_wrist = (wrist_front_thumb + wrist_back_pinky) / 2
    # Orentation of wrist
    y_orient_wrist = (wrist_back_pinky - wrist_front_thumb) / np.linalg.norm(
        wrist_back_pinky - wrist_front_thumb)
    z_orient_wrist = np.cross(hand - wrist_front_thumb, y_orient_wrist)
    z_orient_wrist = z_orient_wrist / np.linalg.norm(z_orient_wrist)
    x_orient_wrist = np.cross(y_orient_wrist, z_orient_wrist)
    x_orient_wrist = x_orient_wrist / np.linalg.norm(x_orient_wrist)
    R_bw = np.array(
        [x_orient_wrist, y_orient_wrist, z_orient_wrist]
    ).T
    R_bw = R_bw @ R_wwr_const
    ''' Pose of wrist w.r.t shoulder '''
    R_sw = R_bs_T @ R_bw
    # Vector pointing from shoulder to wrist in base frame
    p_sw_b = p_wrist - p_shoulder
    T_sb = np.linalg.inv(
        np.block([[R_bs_T.T, p_shoulder.reshape(3, 1)], [np.array([0, 0, 0, 1]).reshape(1, 4)]]))
    # Vector pointing from shoulder to wrist in shoulder frame
    p_sw_s = T_sb @ np.append(p_sw_b, 0)
    p_sw_s = p_sw_s[:3]
    T_sw = np.block([[R_sw, p_sw_s.reshape(3, 1)], [np.array([0, 0, 0, 1]).reshape(1, 4)]])
    # Scale the desired position
    position_unit = T_sw[:3, 3] / np.linalg.norm(T_sw[:3, 3])
    T_sw[:3, 3] = length_scaler * (T_sw[:3, 3] - 0.15 * position_unit) + 0.3 * position_unit
    ''' Norminal vector of arm plane '''
    p_se_b = p_elbow - p_shoulder
    p_se_s = T_sb @ np.append(p_se_b, 0)
    p_se_s = p_se_s[:3]
    n_s_arm = np.cross(p_se_s, p_sw_s)
    n_s_arm = n_s_arm / np.linalg.norm(n_s_arm)
    ''' Swivel Angle'''
    # n_s_v = np.cross(np.array([0.0, -1.0, 0.0]), p_sw_s)
    # n_s_v = n_s_v / np.linalg.norm(n_s_v)
    # sign_swivel = -1
    # if n_s_arm[1] < 0:
    #    sign_swivel = 1
    # swivel_angle = sign_swivel * np.arccos(np.minimum(np.dot(n_s_v, n_s_arm), 1.0))
    # return T_sw, n_s_arm, swivel_angle
    return T_sw, n_s_arm
    


