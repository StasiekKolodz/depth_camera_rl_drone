from gymnasium.envs.registration import register

register(
    id='depth_navigation-v0',
    entry_point='gym_depth_camera_drone.envs:DepthCameraDroneNavigation_v0',
)