#from detection.getTowerMatrix import TowerMatrix 
from controller.pressure import PressureSensor
from detection.getTowerMatrix import TowerMatrix
from controller.controller import Brain

def main():
    
    matrix = TowerMatrix()

    brain =  Brain(matrix)

    pressure = PressureSensor("Gb3", "192.168.0.25")

    while True:
        """ controll loop """

        # get next Matrix
        #matrix.getNextMatrix()

        current_weight = pressure.get()
    
        # get next action


        # do next action

    

main()