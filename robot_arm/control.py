import bot_control
ctrl = bot_control.BotControl()

# all functions are blocking except poke_piece
ctrl.home()                 # drive to homing height
ctrl.descend_to_poke(0, 5)  # descend before piece 0 at line 5
ctrl.poke_piece(0, 5)       # move poker forwards
# TODO: stop poker when too much force is detected
# if force > 10:
#     ctrl.stop()
ctrl.retract_poke(0, 5)     # go back to pre-poke position
ctrl.home()                 # go back over tower
ctrl.grab_piece(0, 5)       # grab the piece from the other side
ctrl.home()                 # bring piece up
ctrl.place_piece(1, 10)     # place piece on top of tower, in the middle position

