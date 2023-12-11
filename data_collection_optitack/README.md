The kinematic verification procedure uses the OptiTrack Motive motion capture system in the Soft Robotics Lab (Unity Hall 100E). To run verification tests, you will need a laptop with MATLAB and OptiTrack Motive 3.0.2 installed. You will also need to disable any antivirus software (McAfee/Norton) to be able to stream data. Make sure to disable both Real Time Scanning and Firewall protection. 


# Installing Software

1. Install MATLAB using [this guide](https://hub.wpi.edu/article/805/matlab-install-guide) from WPI. This code has been tested with MATLAB 2022b, but any recent version should work.

2. Install OptiTrack Motive 3.0.2 using [this link](https://optitrack.com/support/downloads/motive.html). The installer will lead you through downloading needed drivers. Follow defaults.

# Setting Up OptiTrack

<ol>
  <li>
    <p>Set up two additional cameras on tripods such that they are facing your intended workspace. Connect these cameras to the network box with provided Ethernet cables.</p>
  </li>
  <li>
    <p>Plug the license key into your laptop. Plug the Ethernet cable into your laptop. Open Motive.
The first time you open it, you will need to select License Tool and then input the following information:</p>
    <table>
    <thead>
    <tr>
        <th>Name</th>
        <th>Value</th>
    </tr>
    </thead>
    <tbody>
    <tr>
        <td>License_Serial Number</td>
        <td>MTRL1757</td>
    </tr>
    <tr>
        <td>License_Hash</td>
        <td>320D-FDDF-A12C-90B6</td>
    </tr>
    <tr>
        <td>Security key Serial Number</td>
        <td>476789</td>
    </tr>
    <tr>
        <td>Email Address</td>
        <td>schiang@wpi.edu</td>
    </tr>
    <tr>
        <td>First name</td>
        <td>Shou-Shan</td>
    </tr>
    <tr>
        <td>Last Name</td>
        <td>Chiang</td>
    </tr>
    </tbody>
    </table>
  </li>
  <li>
    <p>Click New Calibration on the tab to the left. Clear your workspace of all objects. If you have already taken out the actuator, hide it under the table. Once you have cleared the workspace and everyone has stepped back to the perimeter of the room, select <code class="language-plaintext highlighter-rouge">Mask all visible objects</code>.</p>
  </li>
  <li>
    <p>Set the actuator back on the table. Attach the orange custom ground plane to the actuator. Make sure there are reflective balls pressed onto the piece. Remove any other reflective balls from view of the cameras. Select <code class="language-plaintext highlighter-rouge">Custom Ground Plane</code> and <code class="language-plaintext highlighter-rouge">CTRL + click</code> on each of the three reflective balls in the ground plane. Set the ground plane. If you move the actuator in any following steps, even slightly, you will need to repeat this procedure before you begin testing. Sometimes it is best to set up the actuator before setting the ground plane.</p>
  </li>
  <li>
    <p>There will now be a prompt to <code class="language-plaintext highlighter-rouge">Start Wanding</code>. Select this button and use the calibration wand (usually stored above the camera network box) to calibrate the cameras. Use a “window cleaning” motion to move the wand in large strokes across the field of view of all six cameras. Motive will tell you how many samples each camera has collected. Aim for about 3000-5000 samples per camera. Select <code class="language-plaintext highlighter-rouge">Start Calculating</code> once you are satisfied with the calibration. It will give you the results for this calibration. If it says these results are “poor” you should redo the wanding procedure. Aim for “excellent” or “exceptional” calibrations to get reliable data. Make sure to record your final calibration results in a spreadsheet for later reference.</p>
  </li>
</ol>

# Setting up the Actuator

1. Connect all the motors to the control board. The linear or translational motors go (from outermost tube to innermost tube) to the X, Y, and Z connections, and the rotational motors go to the A, B, C connections. 

2. Insert the tubes into the collets. Tubes need to have square end pieces on the straight part of the tube for the collets to work properly. Insert the innermost tube first. Insert the square piece into the collet and rotate 45 degrees. Use a small locking piece to hold the square in place. Repeat for other tubes. 

# Setting the Home Position for Tube Translation

1. We first want to send the tubes to their home configuration. In the home configuration, all tubes should be curved straight upwards and the ends of all tubes should be flush. The start of the curvature for the outer tube should be in line with the reflective balls that set the ground plane. 

2. Use the “move.m” script to move the robot to the desired location. It works best to establish a single connection (run the top two lines only once). The file is separated out into sections to allow you to run only a few lines of code at a time. You can also copy and paste those lines into the MATLAB terminal if this is easier. Use the code to send positional commands to the robot and to set new positions as the home configuration. 

3. Use the outer tube jig (rectangular orange piece with a cutout for the outer tube) to align the outer tube with the actuator. Set this as a home position. 

# Adjusting the Inner Tube Rotation

1. Extend the inner tube as far out as possible with respect to the outer tube. Place the outer tube collar (orange piece that resembles a plus sign and has four divots for small reflective balls) around the outer tube. Make sure there are four small reflective balls in the piece. If you have moved the actuator at all during setup, reset the ground plane now. 

2. Create a rigid body with the outer tube collar. To do this, select all four circles in Motive, right click, and select `Create Rigid Body`. 

3. Place the end effector (T-shaped piece) in the end-effector holder on the actuator.  Create a rigid body from this as well.  

4. Run the `optitrack_init.m`9 script. You may need to change the DLL Path to reflect where the MATLAB DLL file is located on your computer. You may also need to enable streaming in Motive. Copy the IP address from Motive into the `optitrack_init.m` file. Run this file once to connect to OptiTrack. If you clear your workspace, you will need to run this file again. 

5. Run the `optitrack_read_rigidbody_frame.m` file. This will store the data from the outer tube collar and the end effector rigid bodies in your workspace. You can run `rigidbody{1}.z` to get the Z coordinate of the outer tube collar and `rigidbody{2}.z` to get the Z coordinate of the end effector. If these two coordinates do not match, you will need to use the `move.m` script to rotate the inner tube until they do match. Continue rotating the inner tube and getting measurements from OptiTrack until the two coordinates match. This means that the inner tube does not have any initial rotation with respect to the outer tube. Retract the innermost tube to its 0 position. Set this position as your new home position. 

6. The actuator is now set up and ready for data collection. If you have moved the actuator at all during this procedure, remember to reset the ground plane.

# Running the Verification Script

1. Open the `main.m` file. Make sure to change the COM port to reflect the COM port that the actuator is connected to. 

2. Stand back away from the table and around the perimeter of the room. 

3. Run the `main.m` file to record data. Make sure the actuator is moving the tubes to all desired positions. 

4. Run the `disconnect_optitrack.m` at the end of the session to close all connections.

5. Record all details of the test in a spreadsheet (tube numbers used, type of test, etc.).
