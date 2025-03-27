# Instructions to run reading anxiety study (Android tablet and Misty robot required)

Run the following commands in separate terminal windows
      1. **Setting up dependencies**
  
         cd ~/catkin_ws && catkin_make
         
         source devel/setup.bash
         
         roscore
         
      2. **If running the robot condition, run the following commands to connect to Misty:**
         
          source devel/setup.bash
         
          rosrun misty_api main.py 192.168.1.XXX (enter IP address of Misty robot)
         
      3. **To connect with the tablet run:** ( Note: computer ip address must be entered into the tablet prior to running this section)
         
         source devel/setup.bash
         
         rosrun misty_reading tablet_node.py
         
      4. **Start correction manager** 
         
         source devel/setup.bash
         
         rosrun misty_reading correction_manager.py <storyno1> <storyno2>
    
      5. **Start trascription** 
         
         source devel/setup.bash
         
         rosrun misty_reading transcription_node.py
      
      6. **Record audio**
         
         arecord participantID.wav
