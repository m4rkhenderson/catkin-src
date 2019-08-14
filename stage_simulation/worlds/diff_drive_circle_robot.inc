define laser ranger
(
  sensor
  (
    # range [0.1 25]
    fov 360.0
    # samples 1066
    range [0.1 6]
    # fov 240.0
    # samples 120
    samples 360
    color_rgba [ 0 1 0 1 ] # laser color
  )
  # generic model properties
  #block(
  #  points 4
  #  point[0] [0 1]
  #  point[1] [1 1]
  #  point[2] [1 0]
  #  point[3] [0 0]
  #  z [0 1]
  #)
  block
    (
      points 3
      point[0] [1 0.5]
      point[1] [0 1]
      point[2] [0 0]
      z [0 1]
  )
  color "black"
  #size [ 0.1 0.06 0.1 ]
  size [ 0.4 0.25 0.1 ]
  origin [ 0.0 0.0 0.0 0.0 ]
)

define diff_drive_circle_robot position
(
  size [0.5 0.5 1.2]
  pose [ 0.0 0.0 0.0 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  # This block approximates the circular shape of a robot
  block
  ( 
		points 16
		point[0]  [ 0.225 0.000 ]
		point[1]  [ 0.208 0.086 ]
		point[2]  [ 0.159 0.159 ]
		point[3]  [ 0.086 0.208 ]
		point[4]  [ 0.000 0.225 ]
		point[5]  [ -0.086 0.208 ]
		point[6]  [ -0.159 0.159 ]
		point[7]  [ -0.208 0.086 ]
		point[8]  [ -0.225 0.000 ]
		point[9]  [ -0.208 -0.086 ]
		point[10] [ -0.159 -0.159 ]
		point[11] [ -0.086 -0.208 ]
		point[12] [ -0.000 -0.225 ]
		point[13] [ 0.086 -0.208 ]
		point[14] [ 0.159 -0.159 ]
		point[15] [ 0.208 -0.086 ]
		z [0 1]
  )
	
  color "magenta"
  gui_nose 1 # draw a nose on the model showing its heading (positive X axis)
  obstacle_return 1 # collides with other models
  drive "diff"
  localization "gps" # Report error-free position in world coordinates
  #localization "odom" # Position data drifts from the ground truth over time
  # odometry error model parameters, only used if localization is set to "odom"
  #odom_error [0.03 0.03 999999 999999 999999 0.02]
  #odom_error [0.01 0.01 0.00 0.01] # proportion of error
  # spawn sensors
  # direction(pose [ 0.075 0.0 0.0 0.0 ])
  laser(pose [ 0.0 0.0 0.0 0.0 ])
)

