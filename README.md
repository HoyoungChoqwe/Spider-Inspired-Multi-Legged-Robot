# Spider-Inspired Multi-Legged Robot

A MuJoCo simulation of a spider-inspired eight-legged robot designed to traverse uneven and inclined terrain using coordinated leg motion.

This project was developed for ECE 216 at the University of California, Santa Cruz. It investigates how a low body profile, multiple ground-contact points, and an alternating tetrapod gait can improve stability while climbing uneven slopes.

## Project Overview

Legged robots often experience instability, slipping, and loss of traction when moving across inclined or irregular terrain. This project explores a biologically inspired approach based on spider locomotion.

The simulated robot uses eight articulated legs arranged around a rectangular body. Its gait coordinates the legs into two alternating groups so that several legs remain in contact with the terrain during movement.

The project was developed entirely in simulation using MuJoCo and Python.

## Demonstration

[View the simulation video](ECE216_Simulation.mov)

The demonstration shows the robot moving across the simulated terrain using its alternating leg sequence.

## Project Goals

The main objectives of the project were to:

- Create an eight-legged spider-inspired robot in MuJoCo
- Implement coordinated locomotion across all eight legs
- Maintain a low and stable body posture
- Traverse inclined and uneven terrain
- Reduce excessive slipping and body rotation
- Evaluate the robot’s movement using simulation data

## Robot Design

The robot consists of:

- A rectangular central body
- Eight articulated legs
- Four legs on each side of the body
- Individually actuated leg joints
- A low center of mass for improved stability
- Symmetrical leg placement to reduce unintended turning

The front and rear leg orientations were adjusted to provide better forward propulsion and a wider support pattern.

## Alternating Tetrapod Gait

The locomotion controller divides the legs into two alternating groups.

While one group performs the forward recovery motion, the other group remains in contact with the ground and produces the pushing motion. The groups then exchange roles.

This gait was selected because it allows multiple legs to support the robot at the same time, improving stability compared with moving individual legs independently.

The gait was developed to provide:

- Alternating support between two groups of four legs
- Coordinated movement between the left and right sides
- Leg lifting during the forward stroke
- Ground contact during the backward pushing stroke
- Smooth transitions between gait phases
- Symmetrical movement to reduce turning

## Simulation Environment

The MuJoCo environment includes:

- Gravity
- Contact forces
- Friction
- A solid ground plane
- Inclined terrain
- Uneven surface obstacles
- A flat region at the top of the slope

Terrain dimensions, slope angle, friction, and obstacle placement were adjusted throughout development to evaluate the robot’s stability under different conditions.

## Control Development

The locomotion controller was developed iteratively.

Development included:

- Testing individual leg motion
- Testing paired leg movement
- Coordinating all eight legs
- Dividing the legs into alternating gait groups
- Adjusting joint amplitudes
- Increasing the leg lifting height
- Correcting mirrored leg directions
- Reducing unintended rotation
- Improving forward movement
- Tuning gait speed and timing

Several gait versions were tested before selecting the final coordinated movement pattern.

## Performance Analysis

The simulation was designed to record several measurements:

- Body position over time
- Body pitch over time
- Body height over time
- Slip events across different terrain conditions

These measurements were used to evaluate forward progress, body stability, terrain clearance, and traction.

## Key Challenges

### Symmetrical Movement

Small differences between the left and right leg commands caused the robot to rotate rather than move forward. Joint directions and phase timing were adjusted to improve symmetry.

### Ground Contact

Early leg designs caused the feet to penetrate the floor or remain above the terrain. Leg geometry, body height, and joint ranges were modified to maintain more realistic contact.

### Slipping

Insufficient ground contact and poorly timed leg motion caused excessive sliding. Friction parameters, gait timing, and the pushing phase were adjusted to improve traction.

### Uneven-Terrain Stability

Obstacles and slope transitions disturbed the robot’s posture. A wider support pattern and slower alternating gait helped the robot maintain stability.

## Technologies Used

- Python
- MuJoCo
- XML robot modeling
- Matplotlib
- Robotic locomotion
- Multi-leg gait coordination
- Physics simulation
- Data collection and analysis

## Repository Structure

- `simulation.py` contains the simulation controller, gait logic, data collection, and plotting functions.
- `spider_mujoco.xml` defines the robot model, joints, actuators, terrain, and simulation environment.
- `spider_robot_demo.mov` contains a demonstration of the simulated robot.
- Generated plots show the robot’s position, pitch, body height, and slipping behavior.

Update the names above if the files in the repository use different filenames.

## Running the Simulation

Install MuJoCo and the required Python packages before running the project.

From the project directory, run:

`mjpython simulation.py`

The exact command may differ depending on the local MuJoCo and Python installation.

## Results

The final simulation demonstrated:

- Coordinated motion across eight legs
- Alternating tetrapod locomotion
- Forward movement on inclined terrain
- Improved stability through multiple ground-contact points
- Reduced unintended turning through symmetrical leg commands
- Collection of body-position and stability measurements

## My Contributions

This was an individual simulation project. My work included:

- Designing the spider-inspired robot structure
- Creating the MuJoCo XML model
- Developing the eight-leg gait controller
- Implementing the alternating tetrapod gait
- Designing the inclined and uneven simulation terrain
- Debugging joint orientation and leg movement
- Adjusting friction and contact behavior
- Tuning gait amplitude, speed, and phase timing
- Collecting simulation measurements
- Generating performance graphs
- Evaluating stability and slipping behavior

## Future Improvements

Potential improvements include:

- Closed-loop body stabilization
- Foot-contact sensing
- Adaptive gait timing
- Terrain-aware step placement
- Improved slip detection
- Optimization of gait parameters
- Testing additional terrain profiles
- Comparison with other multi-legged gait patterns
