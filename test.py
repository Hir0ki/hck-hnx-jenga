import URInterface_V0_3
import logging
import traceback
import time

bot = URInterface_V0_3.URController("192.168.0.11")

bot.start_cyclic_reading_session()

digital_outputs = [0, 0, 0, 0]

def set_outputs():
    bot.set_digital_outputs(digital_outputs)

def close_gripper():
    bot.set_digital_output(0, 1)
    bot.set_digital_output(1, 0)
    # digital_outputs[0] = 1
    # digital_outputs[1] = 0
    # set_outputs()

def open_gripper():
    bot.set_digital_output(0, 0)
    bot.set_digital_output(1, 1)
    # digital_outputs[0] = 0
    # digital_outputs[1] = 1
    # set_outputs()

try:
    running = True
    print("Init done...")
    while running:
        # print("coords", bot.get_actual_tool_coordinates())
        # print("accels", bot.get_actual_tool_accelerations())
        # print("velocs", bot.get_actual_tool_velocities())
        bot.move_relative([0, 0.01, 0, 0, 0, 0])
        time.sleep(1)
        # open_gripper()
        # time.sleep(1)
        # close_gripper()
except Exception as e:
    running = False
    logging.error(traceback.format_exc())
    bot.close_cyclic_reading_session()
    del bot

