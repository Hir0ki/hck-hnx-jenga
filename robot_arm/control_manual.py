import logging
import traceback
import threading
import time

import bot_control
ctrl = bot_control.BotControl()
# ctrl = None

x = None
z = None

running = True



try:
    while(running):

        print("""<Enter> Stops robot
    01. Set piece coordinates (eg. "0,1")
    02. home
    03. descend_to_poke
    04. poke_piece
    05. retract_poke
    06. grab_piece
    07. place_piece (attention: set new coords before)
        """)

        sel = input("Select option: ")

        if(sel == ""):
            ctrl.stop()
            pass
        else:
            sel = int(sel)

            if(sel == 1):
                input_str = input("X, Z: ")
                lst = input_str.split(",")
                print(lst)
                z = int(lst[1])
                x = int(lst[0])
            elif(sel == 2):
                ctrl.home()
            elif(sel == 3):
                ctrl.descend_to_poke(x, z)
            elif(sel == 4):
                ctrl.poke_piece(x, z)
            elif(sel == 5):
                ctrl.retract_poke(x, z)
            elif(sel == 6):
                ctrl.grab_piece(x, z)
            elif(sel == 7):
                ctrl.place_piece(x, z)

except Exception as e:
    running = False
    logging.error(traceback.format_exc())

