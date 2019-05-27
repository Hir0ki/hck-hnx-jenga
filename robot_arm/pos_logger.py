
import time

import bot_control
ctrl = bot_control.BotControl(False)

running2 = True
while (running2):
    try:
        print(["{0:0.4f}".format(i)
                    for i in ctrl.bot.get_pose().pose_vector])
        time.sleep(1)
    except Exception as e:
        running2 = False