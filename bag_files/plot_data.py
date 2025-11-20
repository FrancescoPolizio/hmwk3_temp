import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition, ManualControlSetpoint, ActuatorOutputs

# Path to bag folder
BAG_PATH = '/home/user/ros2_ws/src/bag_files/data/' 

def read_bag_data(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Could not open as sqlite3, trying mcap... ({e})")
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        reader.open(storage_options, converter_options)

    # Data Containers
    pos_time, pos_z_enu = [], []
    rc_time, rc_throttle = [], []
    act_time, act_m1, act_m2, act_m3, act_m4 = [], [], [], [], []

    print("Reading messages...")
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        time_sec = t / 1e9 

        if topic == '/fmu/out/vehicle_local_position':
            msg = deserialize_message(data, VehicleLocalPosition)
            pos_time.append(time_sec)
            pos_z_enu.append(-msg.z) # Convert NED z to ENU z 
            
        elif topic == '/fmu/out/manual_control_setpoint':
            msg = deserialize_message(data, ManualControlSetpoint)
            rc_time.append(time_sec)
            rc_throttle.append(msg.throttle)

        elif topic == '/fmu/out/actuator_outputs':
            msg = deserialize_message(data, ActuatorOutputs)
            act_time.append(time_sec)
            # Check if output array is large enough
            if len(msg.output) >= 4:
                act_m1.append(msg.output[0])
                act_m2.append(msg.output[1])
                act_m3.append(msg.output[2])
                act_m4.append(msg.output[3])

    return (pos_time, pos_z_enu), (rc_time, rc_throttle), (act_time, act_m1, act_m2, act_m3, act_m4)

def plot_data(pos_data, rc_data, act_data):
    (p_time, p_z) = pos_data
    (r_time, r_thr) = rc_data
    (a_time, a_m1, a_m2, a_m3, a_m4) = act_data

    #Start Time (The earliest timestamp found across ANY topic)
    start_candidates = []
    if p_time: start_candidates.append(p_time[0])
    if r_time: start_candidates.append(r_time[0])
    if a_time: start_candidates.append(a_time[0])

    if not start_candidates:
        print("CRITICAL: No data found in bag for the requested topics.")
        return

    start_time = min(start_candidates)

    #  Normalize Times
    if p_time: p_time = [t - start_time for t in p_time]
    if r_time: r_time = [t - start_time for t in r_time]
    if a_time: a_time = [t - start_time for t in a_time]

    # Plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

    # Plot 1: Altitude
    if p_time:
        ax1.plot(p_time, p_z, 'b-', label='Altitude (ENU)')
        ax1.axhline(y=20, color='r', linestyle='--', label='Threshold (20m)')
        ax1.legend(loc='upper right')
    else:
        ax1.text(0.5, 0.5, "NO POSITION DATA RECORDED\nCheck topic name", ha='center', transform=ax1.transAxes, color='red')
    ax1.set_ylabel('Height [m]')
    ax1.set_title('Drone Flight Analysis')
    ax1.grid(True)

    # Plot 2: RC Input
    if r_time:
        ax2.plot(r_time, r_thr, 'g-', label='Pilot Throttle')
        ax2.legend(loc='upper right')
    else:
        ax2.text(0.5, 0.5, "No Manual Control Data", ha='center', transform=ax2.transAxes)
    ax2.set_ylabel('Stick Input')
    ax2.grid(True)

    # Plot 3: Actuators
    if a_time:
        ax3.plot(a_time, a_m1, label='M1', alpha=0.7)
        ax3.plot(a_time, a_m2, label='M2', alpha=0.7)
        ax3.plot(a_time, a_m3, label='M3', alpha=0.7)
        ax3.plot(a_time, a_m4, label='M4', alpha=0.7)
        ax3.legend(loc='upper right', ncol=4)
    else:
        ax3.text(0.5, 0.5, "No Actuator Data", ha='center', transform=ax3.transAxes)
    ax3.set_ylabel('Motor Output')
    ax3.set_xlabel('Time [s]')
    ax3.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    position_data, pilot_data, actuator_data = read_bag_data(BAG_PATH)
    plot_data(position_data, pilot_data, actuator_data)
