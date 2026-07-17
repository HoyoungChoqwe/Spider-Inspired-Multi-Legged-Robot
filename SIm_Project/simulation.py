import time
import csv
import numpy as np
import mujoco
import mujoco.viewer

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


model = mujoco.MjModel.from_xml_path("spider_mujoco.xml")
data = mujoco.MjData(model)


MASS = 0.65
GRAVITY = 9.81
SLOPE_DEG = 20.0
MU = 4.0
LOSS_FACTOR = 0.20

theta = np.deg2rad(SLOPE_DEG)

Fn = MASS * GRAVITY * np.cos(theta)
Fg_x = MASS * GRAVITY * np.sin(theta)
Ff_max = MU * Fn
F_loss = LOSS_FACTOR * Fg_x

TARGET_UPHILL_SPEED = 0.22

KP_SPEED = 12.0
KD_SIDE = 16.0
KD_VERTICAL = 2.0

MAX_UPHILL_FORCE = 4.5
MAX_SIDE_FORCE = 2.0
MAX_VERTICAL_FORCE = 1.0

# Increase this if you want the spider to walk farther before stopping.
STOP_X = 7.40

BRAKE_X = 10.0
BRAKE_Y = 10.0
BRAKE_Z = 2.0
MAX_BRAKE_FORCE = 5.0

# ------------------------------------------------------------
# Graph / data logging settings
# ------------------------------------------------------------

LOG_DT = 0.05

# Slip is counted only when contacting feet move unusually fast.
# Higher value = fewer slip counts.
SLIP_SPEED_THRESHOLD = 0.60

# Prevents one slip event from being counted many times.
SLIP_COOLDOWN_TIME = 0.80


def d_shape_motion(phase, yaw_amp, lift_amp):
    if phase < 0.5:
        s = phase / 0.5
        yaw = yaw_amp * np.cos(np.pi * s)
        pitch = lift_amp * np.sin(np.pi * s)
    else:
        s = (phase - 0.5) / 0.5
        yaw = -yaw_amp + 2.0 * yaw_amp * s
        pitch = -0.06

    return yaw, pitch


def set_leg(ctrl_index, yaw, pitch, yaw_sign, pitch_sign):
    data.ctrl[ctrl_index] = yaw_sign * yaw
    data.ctrl[ctrl_index + 1] = pitch_sign * pitch


def set_all_legs_neutral():
    for i in range(model.nu):
        data.ctrl[i] = 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


def get_body_pitch_deg():
    """
    Free joint qpos layout:
    qpos[0:3] = x, y, z
    qpos[3:7] = quaternion w, x, y, z
    """
    qw = data.qpos[3]
    qx = data.qpos[4]
    qy = data.qpos[5]
    qz = data.qpos[6]

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    sin_pitch = clamp(sin_pitch, -1.0, 1.0)

    pitch_rad = np.arcsin(sin_pitch)
    return np.rad2deg(pitch_rad)


def save_csv(times, x_positions, pitches, heights, slip_counts):
    with open("simulation_data.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            "body_x_position_m",
            "body_pitch_deg",
            "body_height_m",
            "slip_count"
        ])

        for i in range(len(times)):
            writer.writerow([
                times[i],
                x_positions[i],
                pitches[i],
                heights[i],
                slip_counts[i]
            ])


def save_single_plot(x, y, title, xlabel, ylabel, filename):
    plt.figure(figsize=(8, 5))
    plt.plot(x, y, linewidth=2)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close()


def save_all_plots(times, x_positions, pitches, heights, slip_counts):
    save_csv(times, x_positions, pitches, heights, slip_counts)

    save_single_plot(
        times,
        x_positions,
        "Body X Position Over Time",
        "Time (s)",
        "Body X Position (m)",
        "body_x_position.png"
    )

    save_single_plot(
        times,
        pitches,
        "Body Pitch Over Time",
        "Time (s)",
        "Body Pitch (deg)",
        "body_pitch.png"
    )

    save_single_plot(
        times,
        heights,
        "Body Height Over Time",
        "Time (s)",
        "Body Height Z (m)",
        "body_height.png"
    )

    save_single_plot(
        times,
        slip_counts,
        "Slip Count Against Terrain Over Time",
        "Time (s)",
        "Cumulative Slip Count",
        "slip_count.png"
    )

    print("\nSaved graph files:")
    print("body_x_position.png")
    print("body_pitch.png")
    print("body_height.png")
    print("slip_count.png")
    print("simulation_data.csv")


def get_geom_id(name):
    return mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        name
    )


spider_body_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "spider_body"
)

# Foot geom IDs
foot_names = [
    "L1_foot", "L2_foot", "L3_foot", "L4_foot",
    "R1_foot", "R2_foot", "R3_foot", "R4_foot"
]

foot_geom_ids = [get_geom_id(name) for name in foot_names]

# Terrain geom IDs: slope, platform, and all obstacles that exist in the XML
terrain_geom_ids = set()

for name in ["main_slope", "top_platform"]:
    gid = get_geom_id(name)
    if gid != -1:
        terrain_geom_ids.add(gid)

for i in range(1, 100):
    gid = get_geom_id(f"obs_{i}")
    if gid != -1:
        terrain_geom_ids.add(gid)

for i in range(1, 100):
    gid = get_geom_id(f"edge_obs_{i}")
    if gid != -1:
        terrain_geom_ids.add(gid)


# Logging arrays
times = []
x_positions = []
pitches = []
heights = []
slip_counts = []

slip_count = 0
last_log_time = 0.0
previous_foot_positions = None
last_slip_time = -999.0


with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = spider_body_id
    viewer.cam.distance = 3.2
    viewer.cam.azimuth = -135
    viewer.cam.elevation = -25

    last_print_time = 0.0
    last_x_pos = data.qpos[0]
    last_x_vel = 0.0

    reached_top = False

    while viewer.is_running():
        t = time.time() - start_time

        data.xfrc_applied[:, :] = 0.0

        x_pos = data.qpos[0]
        vx = data.qvel[0]
        vy = data.qvel[1]
        vz = data.qvel[2]

        if x_pos >= STOP_X:
            reached_top = True

        if reached_top:
            set_all_legs_neutral()

            brake_fx = clamp(-BRAKE_X * vx, -MAX_BRAKE_FORCE, MAX_BRAKE_FORCE)
            brake_fy = clamp(-BRAKE_Y * vy, -MAX_BRAKE_FORCE, MAX_BRAKE_FORCE)
            brake_fz = clamp(-BRAKE_Z * vz, -MAX_VERTICAL_FORCE, MAX_VERTICAL_FORCE)

            data.xfrc_applied[spider_body_id, 0] = brake_fx
            data.xfrc_applied[spider_body_id, 1] = brake_fy
            data.xfrc_applied[spider_body_id, 2] = brake_fz

        else:
            freq = 0.35

            yaw_amp = 0.26
            lift_amp = 0.32

            phase_a = (freq * t) % 1.0
            phase_b = (phase_a + 0.5) % 1.0

            yaw_a, pitch_a = d_shape_motion(phase_a, yaw_amp, lift_amp)
            yaw_b, pitch_b = d_shape_motion(phase_b, yaw_amp, lift_amp)

            # Group A: L1, R2, L3, R4
            set_leg(0,  yaw_a, pitch_a, +1, +1)
            set_leg(10, yaw_a, pitch_a, -1, -1)
            set_leg(4,  yaw_a, pitch_a, +1, +1)
            set_leg(14, yaw_a, pitch_a, -1, -1)

            # Group B: R1, L2, R3, L4
            set_leg(8,  yaw_b, pitch_b, -1, -1)
            set_leg(2,  yaw_b, pitch_b, +1, +1)
            set_leg(12, yaw_b, pitch_b, -1, -1)
            set_leg(6,  yaw_b, pitch_b, +1, +1)

            speed_error = TARGET_UPHILL_SPEED - vx
            uphill_force = KP_SPEED * speed_error + Fg_x + F_loss
            uphill_force = clamp(uphill_force, 0.0, MAX_UPHILL_FORCE)

            side_force = -KD_SIDE * vy
            side_force = clamp(side_force, -MAX_SIDE_FORCE, MAX_SIDE_FORCE)

            vertical_force = -KD_VERTICAL * vz
            vertical_force = clamp(vertical_force, -MAX_VERTICAL_FORCE, MAX_VERTICAL_FORCE)

            data.xfrc_applied[spider_body_id, 0] = uphill_force
            data.xfrc_applied[spider_body_id, 1] = side_force
            data.xfrc_applied[spider_body_id, 2] = vertical_force

        mujoco.mj_step(model, data)

        # ------------------------------------------------------------
        # Data logging for graphs
        # ------------------------------------------------------------
        if t - last_log_time >= LOG_DT:
            current_foot_positions = {}

            for foot_id in foot_geom_ids:
                if foot_id != -1:
                    current_foot_positions[foot_id] = np.copy(data.geom_xpos[foot_id])

            foots_touching_terrain = set()

            for i in range(data.ncon):
                contact = data.contact[i]
                g1 = contact.geom1
                g2 = contact.geom2

                if g1 in foot_geom_ids and g2 in terrain_geom_ids:
                    foots_touching_terrain.add(g1)

                if g2 in foot_geom_ids and g1 in terrain_geom_ids:
                    foots_touching_terrain.add(g2)

            if previous_foot_positions is not None:
                dt_log = t - last_log_time

                slipping_feet = 0

                for foot_id in foots_touching_terrain:
                    if foot_id in previous_foot_positions and foot_id in current_foot_positions:
                        previous_pos = previous_foot_positions[foot_id]
                        current_pos = current_foot_positions[foot_id]

                        foot_velocity = (current_pos - previous_pos) / dt_log

                        # Horizontal foot speed while touching terrain
                        horizontal_speed = np.sqrt(
                            foot_velocity[0] ** 2 + foot_velocity[1] ** 2
                        )

                        if horizontal_speed > SLIP_SPEED_THRESHOLD:
                            slipping_feet += 1

                # Count only one slip event at a time, not every foot every frame.
                # Also require at least 2 feet slipping to avoid counting normal leg motion.
                if slipping_feet >= 2 and (t - last_slip_time) > SLIP_COOLDOWN_TIME:
                    slip_count += 1
                    last_slip_time = t

            times.append(t)
            x_positions.append(data.qpos[0])
            pitches.append(get_body_pitch_deg())
            heights.append(data.qpos[2])
            slip_counts.append(slip_count)

            previous_foot_positions = current_foot_positions
            last_log_time = t

        current_x_pos = data.qpos[0]
        dt_print = t - last_print_time

        if dt_print >= 1.0:
            x_vel = (current_x_pos - last_x_pos) / dt_print
            x_acc = (x_vel - last_x_vel) / dt_print

            Fx_net = MASS * x_acc
            F_legs_est = Fx_net + Fg_x + F_loss

            print("\n--- Modeling Equations Estimate ---")
            print(f"theta = {SLOPE_DEG:.1f} deg")
            print(f"Fn = mg cos(theta) = {Fn:.3f} N")
            print(f"Fg,x = mg sin(theta) = {Fg_x:.3f} N")
            print(f"Ff,max = mu Fn = {Ff_max:.3f} N")
            print(f"Floss estimate = {F_loss:.3f} N")
            print(f"Fx,net = m ax = {Fx_net:.3f} N")
            print(f"Flegs estimate = Fx,net + Fg,x + Floss = {F_legs_est:.3f} N")
            print(f"x position = {current_x_pos:.3f} m")
            print(f"x velocity = {x_vel:.3f} m/s")
            print(f"y slip velocity = {vy:.3f} m/s")
            print(f"Reached top platform = {reached_top}")
            print(f"Slip count = {slip_count}")

            last_print_time = t
            last_x_pos = current_x_pos
            last_x_vel = x_vel

        viewer.sync()


# After you close the MuJoCo viewer, graphs are saved.
if len(times) > 1:
    save_all_plots(times, x_positions, pitches, heights, slip_counts)
else:
    print("Not enough data was recorded to make graphs.")