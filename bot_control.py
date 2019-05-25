import urx
import time
import logging
import traceback
import math
import math3d as m3d


rad = math.radians

logging.basicConfig(level=logging.INFO)

class BotControl():
    running = True
    bot = None

    v = 0.05
    a = 0.1

    tower_pos = (0.3713, -0.0330, 0.0414)   # upper corner of Jenga tower
    init_pos = (0.371 - 0.0375, -0.0330 + 0.0375, 0.260)
    jenga_piece = (0.075, 0.025, 0.015)     # dimensions of Jenga piece
    gripper_distance = 0.025                # distance of gripper from tower edge
    poker_distance = 0.1                    # distance of poker from tower
    poker_z_offset = -0.02
    grip_angle_straight = (-0.9218, 2.9975, 0.0018)
    grip_angle_top_left = (-2.5718, 1.4158, -0.3440)
    grip_angle_top_right = (0.8593, -2.9526, 0.4590)
    poke_angle_bottom_right = (-0.8450, 3.0190, -0.0060)
    poke_angle_bottom_left = (1.5281, 2.7426, -0.0118)

    def close_gripper(self):
        self.bot.set_digital_out(0, 1)
        self.bot.set_digital_out(1, 0)

    def open_gripper(self):
        self.bot.set_digital_out(0, 0)
        self.bot.set_digital_out(1, 1)

    def calc_poke_pos(self, x_piece, z_tier):
        pos = list(self.tower_pos)
        orient = list(self.grip_angle_straight)
        if z_tier % 2 == 0:   # even tier
            pos[0] += self.poker_distance
            pos[1] += self.jenga_piece[1] / 2
            pos[1] += self.jenga_piece[1] * x_piece
            orient = list(self.poke_angle_bottom_right)
        else:                   # uneven tier
            pos[1] -= self.poker_distance
            pos[0] -= self.jenga_piece[1] / 2
            pos[0] -= self.jenga_piece[1] * x_piece
            orient = list(self.poke_angle_bottom_left)
        pos[2] += self.jenga_piece[2] * z_tier
        pos[2] += self.poker_z_offset

        pos.extend(orient)
        return pos

    def calc_grab_pos(self, x_piece, z_tier):
        pos = list(self.tower_pos)
        orient = list(self.grip_angle_straight)
        if z_tier % 2 == 0:   # even tier
            pos[0] -= self.jenga_piece[0]
            pos[0] -= self.gripper_distance
            # pos[1] += self.jenga_piece[1] / 2
            pos[1] += 0.006
            pos[1] += self.jenga_piece[1] * x_piece
            # orient = list(self.grip_angle_straight)
            orient = list(self.grip_angle_top_left)
        else:
            pos[1] += self.jenga_piece[0]
            pos[1] += self.gripper_distance
            pos[0] -= self.jenga_piece[1] / 2
            pos[0] -= self.jenga_piece[1] * x_piece
            orient = list(self.grip_angle_top_right)
        pos[2] += self.jenga_piece[2] * z_tier

        pos.extend(orient)
        return pos

    # ALWAYS make sure, that arm is in init_pos, or above tower!
    def grab_piece(self, x, z):
        cur_pose = self.bot.get_pose().pose_vector
        target = self.calc_grab_pos(x, z)

        pos = target[:3]        # get position from target
        orient = target[-3:]    # get orientation from target

        pos[2] = cur_pose[2]    # stay on same z height
        self.move_to(pos, orient)

        pos[2] = target[2]      # descend to target height
        self.move_to(pos, orient)

        self.close_gripper()

        if z % 2 == 0:   # even tier
            pos[0] -= self.jenga_piece[0]
        else:            # odd tier
            pos[1] += self.jenga_piece[0]
        self.move_to(pos, orient)  # pull out piece

    def to_pose(self, pos, orient):
        target = pos.copy()
        target.extend(orient)
        return target

    def move_to(self, to_move_pos, to_move_orient=None, acc=a, vel=v):
        if to_move_orient == None:
            self.bot.set_pos(to_move_pos)
        else:
            to_move = self.to_pose(to_move_pos, to_move_orient)
            self.bot.set_pose(m3d.Transform(to_move), acc, vel, wait=False)
            time.sleep(0.05)
            try:
                self.bot._wait_for_move(to_move, timeout=30)
            except urx.urrobot.RobotException as e:
                print(e)
                print("ERROR, ROBOT STOPPED!!!!")
                print(self.bot.secmon.running)
                print(self.bot.secmon._parser.version)
                print(self.bot.secmon._dict["RobotModeData"])

    # def move_relative(self, relative, acc=self.a, vel=self.v):

    # def set_orient(self, orient):
    #     pos = self.bot.getl()[0:3]
    #     pos.extend(orient)
    #     self.bot.movex("movel", pos, self.a, self.v)

    def main(self):
        while self.running:
            try:
                # print("pos", self.bot.get_pos().list)
                # print("pose", self.bot.get_pose().pose_vector)
                print(["{0:0.4f}".format(i)
                       for i in self.bot.get_pose().pose_vector])
                # print("orient", self.bot.get_orientation().rotation_vector.tolist())
                time.sleep(1)
                # to_move = self.calc_grab_pos(0, 2)
                # print("to_move", to_move)
                # self.bot.set_pose(m3d.Transform(to_move), self.a, self.v)
                # break
                # self.bot.set_orientation(list(self.poke_angle_bottom_left), self.a, self.v)
                # time.sleep(15)
                # self.bot.set_orientation(list(self.grip_angle_top_right), self.a, self.v * 10)
                # time.sleep(15)
                # self.bot.set_orientation(list(self.grip_angle_top_left), self.a, self.v * 10)
                # time.sleep(15)
                # self.bot.set_orientation(list(self.poke_angle_bottom_right), self.a, self.v * 10)
                # time.sleep(15)
                # self.set_orient(to_move[1])
                # self.close_gripper()
                # time.sleep(1)
                # self.open_gripper()
                # self.bot.translate((0, 0.05, 0), self.a, self.v)
                # self.bot.x_t += 0.01
                # time.sleep(0.1)
            except urx.urrobot.RobotException as e:
                print(e)

    def __init__(self):
        try:
            self.bot = urx.Robot("192.168.0.11")
            self.bot.secmon.running = True
            self.bot.secmon._parser.version = (3, 0)
            print(self.bot.secmon._parser.version)
            # bot.set_tcp((0, 0, 0.999, 0, 0, 0))
            trx = m3d.Transform()
            trx.orient.rotate_zb(math.radians(-20 + 45))
            self.bot.set_csys(trx)
            # self.bot.set_pos(self.tower_pos, self.a, self.v)

        except urx.urrobot.RobotException as e:
            print(e)
        except Exception as e:
            self.running = False
            logging.error(traceback.format_exc())
            if self.bot:
                print("BOT STOPPED")
                self.bot.stop()
                self.bot.close()
            raise Exception("Could not connect to robot")
            return

        self.open_gripper()
        self.move_to(list(self.init_pos),
                        list(self.grip_angle_straight))
        return


if __name__ == "__main__":
    ctrl = BotControl()
    ctrl.main()
