To verify the issue and the behavioral differences:

1.Open the original scene file (original_model.ttt) in CoppeliaSim.

2.Run the main MATLAB script sim_non_realtime.m. Observe the normal operation of the quadruped robot.

3.Close the original scene and open the modified scene file (modified_model.ttt).

4.Run the same MATLAB script sim_non_realtime.m again without any modifications. 

You will observe that the control of the thigh and shank joints for both front legs fails, as described.