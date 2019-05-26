from random import randint
from  robot_arm.bot_control import BotControl
from controller.pressure import PressureSensor
class Brain():

    def __init__(self, _aktive_rows):
        self._aktive_rows = _aktive_rows  
        #self.ctrl = BotControl()  #if robot is aktive
        #self.pressure_sensor = PressureSensor("Gb3", "192.168.0.25")
    

    def get_next_move(self):
        
        print(self._aktive_rows) 
        piece, line = self._get_random_position()

        print('home')
        #self.ctrl.home()
        print('desend')
        #self.ctrl.descend_to_poke(piece, line)

        print('poke')
       # self.ctrl.poke_piece(piece, line)
        
        print('moving')
        #while self.pressure_sensor.get() > 0.5:
        #    print('moving')

        print('retract')
        #self.ctrl.retract_poke(piece, line)
        print('home')
        #self.ctrl.home()

        print('grab')
        #self.ctrl.grab_piece(piece, line)

        # change row status
        self._aktive_rows[line][piece] = False

        print('home')
        #self.ctrl.home()

        print('place')
        #self._get_place_top()

    def _get_random_position(self):
        rowid =  randint( 2, len(self._aktive_rows) - 2 ) 
        entryid = randint(  0 , 2 )

        if self._aktive_rows[rowid][entryid] == False:
           entryid, rowid = self._get_random_position()

        return ( entryid, rowid )

    def _get_place_top(self):
        if len(self._aktive_rows[-1]) <= 3:
            self._aktive_rows[-1].append(True)
        else:
            self._aktive_rows.append([])
            self._aktive_rows[-1] = [True]
        
        piece = len(self._aktive_rows[-1]) - 1
        line = len(self._aktive_rows) - 1

        #self.ctrl.place_piece( piece, line )
