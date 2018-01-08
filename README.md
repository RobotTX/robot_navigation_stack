###Update of new version robot specification

#change username#
username_script.sh

#footprint & other parameters
gobot_base.launch   wheel_separation    middle:0.351  outside:0.402
                    wheel_radius        0.0725 -> 0.075
                    ticks_per_rotation  819.2 -> 980
*costmap.yaml/gmapping.yaml/teb_local.yaml
[[0.2495,0],[0.2478,-0.0835],[0.242,-0.167],[0.2203,-0.2164],[0.1786,-0.2411],[0.0893,-0.2484],[0,-0.25],[-0.0893,-0.2484],[-0.1786,-0.2411],[-0.2203,-0.2164],[-0.242,-0.167],[-0.2478,-0.0835],[-0.2495,0],[-0.2478,0.0835],[-0.242,0.167],[-0.2203,0.2164],[-0.1786,0.2411],[-0.0893,0.2484],[0,0.25],[0.0893,0.2484],[0.1786,0.2411],[0.2203,0.2164],[0.242,0.167],[0.2478,0.0835]]

*TF
-sonar z=0.3-0.4 front_right:0.243,0.12; front_left:0.243,-0.12; back_right:-0.246,0.112; back_left:-0.246,-0.112
-laser z=0.2-0.3 200,0
-cliff z=0-0.1 front_right:0.215,0.18; front_left:0.215,-0.18; back_right:-0.215,0.18; back_left:-0.215,-0.18
-bumpers z=0.1-0.2 [0.125,0.225] [0.01,0.125] [-0.125,-0.01] [-0.225,-0.125]

*costmap_common.yaml/teb_local_planner.yaml
sqrt(0.255^2+0.25^2) = 0.357  
inflation_radius = 0.45     0.4/0.45/0.5  