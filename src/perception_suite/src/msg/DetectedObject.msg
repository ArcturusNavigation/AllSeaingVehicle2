# Detected Object Msg

Header     header               # msg timestamp, frame_id = sensor frame
time       time_detected        # Time sensor detected object
string     sensor_type	     # Source of detection
string     uuid		     # Universally unique identifier

AISContact     ais                  # From AIS (Optional)

geometry_msgs/PoseWithCovariance     pose_from_robot      # (xy) position and (theta) orientation in robot space
geometry_msgs/Point                  bb		     # length, width, height			

