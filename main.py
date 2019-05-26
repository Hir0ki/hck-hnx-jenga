#from detection.getTowerMatrix import TowerMatrix 
from controller.pressure import PressureSensor
from detection.getTowerMatrix import TowerMatrix
from controller.controller import Brain
from detection.pyueye_main import Picture

def main():
    

    camera = Picture()
        
    rows = camera.get_next_frame()
        
    brain =  Brain(rows)

    count = 0

    while count < 4:
        """ controll loop """

        # get next Matrix
        
        print(brain.get_next_move())

        #current_weight = pressure.get()
    
        # get next action
        count += 1

        # do next action

        # shutdown
    camera.shutdown()

main()