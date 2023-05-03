'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        # target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        current_time = perception.time - self.start_time
        names, times, keys = keyframes
        
        for joint_index, name in enumerate(names):
            for time_point in range(len(times[joint_index]) - 1):
                if (times[joint_index][time_point] <= current_time <= times[joint_index][len(times[joint_index]) - 1]):
                    first_point = keys[joint_index][time_point][0]
                    second_point = keys[joint_index][time_point + 1][0]

                    control_point_following_offset = keys[joint_index][time_point][2][2]
                    control_point_preceding_offset = keys[joint_index][time_point][1][2]
                    dtime_following = keys[joint_index][time_point][2][1]
                    dtime_preceding = keys[joint_index][time_point][1][1]

                    control_point_following = first_point + control_point_following_offset * dtime_following
                    control_point_preceding = second_point + control_point_preceding_offset * dtime_preceding

                    t = (current_time - times[joint_index][time_point]) / (times[joint_index][time_point + 1] - times[joint_index][time_point])

                    target_joints[name] = (1-t)**3 * first_point + 3 * t * (1-t)**2 * control_point_following + 3 * t**2 * (1-t) * control_point_preceding + t**3 * second_point

        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
