 fuzzy_controller_system

========================

This is an ongoing project for controlling a quadrotor with fuzzylogic.

Right now we have to main programs.

Fuzzy_lander consist of using the bottom camera to identify a Augmented Reality tag and attemp to land on it. Currently it still needs more work.

Fuzzy_tracker uses the front camera to detect a AR tag, and attempts to keep the tag in the center of the frame. Currently working.

TODO: ROW YAW PITCH Control for both systems

Jackie Ortiz Josue Cruz-Lambert


To RUN
========================

You need to connect to ArDrone 2.0 via wifi.
then start up the ar_track_alavr pacakge
Have a way to control the ardrone manuall, i recommend the tum_drone drone_gui program from Tum_ardrone pkg
then rosrun fuzzy_controller fuzzy_tracker

When you send the takeoff command to the ardrone, the fuzzy controller will start sending out command. Send the land command to stop it.

