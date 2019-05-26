#from detection.getTowerMatrix import TowerMatrix 
from controller.pressure import PressureSensor
from detection.getTowerMatrix import TowerMatrix
from controller.controller import Brain
from detection.pyueye_main import Picture
def main():
    
    matrix = TowerMatrix()

    brain =  Brain(matrix)

    camera = Picture()
        
    #pressure = PressureSensor("Gb3", "192.168.0.25")
    
    while True:
        """ controll loop """

        # get next Matrix
        rows = camera.get_next_frame()
        
        print(rows)


        #current_weight = pressure.get()
    
        # get next action


        # do next action

        # shutdown
        camera.shutdown()

main()