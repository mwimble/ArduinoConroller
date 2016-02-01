
# Desc: Device definitions for Robotnik SummitXL Robot.
# Date 2012-01-07

define summit_laser_front laser
(
#  range_min 0.0
# range_max 5.0    #urg04lx
  range_max 30.0   #utm30lx
  fov 235.0
  samples 270

  # generic model properties
  size [ 0.07 0.07 0.05 ]
#  pose [ 0.315 0.0 -0.15 0 ]
  pose [ 0.335 0.0 -0.1 0 ]
  color "blue"
)

define summit_position position
(
#  color "red"		# Default color.

  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).
  wheelbase 0.37

  # alternative odometric localization with simple error model
  # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   
)

define summit_base  summit_position
(
  # actual size
  size [0.7 0.5 0.3] # sizes

  # the SummitXL's center of rotation is in the center of the robot
  # expressed as displ respect geom center
  origin [0 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 0 

  # estimated mass in KG
  mass 25.0 

  # ackerman type steering drive
  drive "diff"
)

define summit_no_laser summit_base
(
    # rear right wheel
    block
    (
        points 4
        point[0] [ -0.098  -0.079 ]
        point[1] [ -0.348  -0.079 ]
        point[2] [ -0.348  -0.229 ]
    	point[3] [ -0.098  -0.229 ]
    	z [ 0 0.3 ]
    	color "black"
    )

    #front right wheel
    block
    (
        points 4
        point[0] [ 0.348 -0.079 ]
        point[1] [ 0.098 -0.079 ]
        point[2] [ 0.098 -0.229 ]
        point[3] [ 0.348 -0.229 ]
        z [ 0 0.3 ]
        color "black"
    )

    # rear left wheel
    block
    (
        points 4
        point[0] [ -0.098  0.229 ]
        point[1] [ -0.348  0.229 ]
        point[2] [ -0.348  0.079 ]
        point[3] [ -0.098  0.079 ]
        z [ 0 0.3 ]
        color "black"
    )

    #front left wheel
    block
    (
        points 4
        point[0] [ 0.348 0.229 ]
        point[1] [ 0.098 0.229 ]
        point[2] [ 0.098 0.079 ]
        point[3] [ 0.348 0.079 ]
        z [ 0 0.3 ]
        color "black"
    )

 # simplified Body shape:
  # the shape of SummitXL

  # simplified Body shape:
  block(
        points 8
        point[0] [0.07 -0.18]
        point[1] [0.305 -0.112]
        point[2] [0.305 0.112]
        point[3] [0.07 0.18]
        point[4] [-0.27 0.18]
        point[5] [-0.305 0.145]
        point[6] [-0.305 -0.145]
        point[7] [-0.27 -0.18]
        z [ 0 0.3 ]
        color "red"
    )


)

define summit_no_laser summit_base
(
  # The Summit center of rotation is offset from its center of area
  origin [0.0 0.0 0.0 0.0]

  # Estimated mass in KG
  mass 25.0 
) 

define summit_xl summit_no_laser
(
  summit_laser_front()
)
