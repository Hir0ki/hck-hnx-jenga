#from detection.getTowerMatrix import TowerMatrix 
from pressure import PressureSensor

def main():
    
 #   matrix = TowerMatrix()

    pressure = PressureSensor("Gb3", "192.168.0.25")

    while True:
        """ controll loop """
        #matrix.getNextMatrix()

        print(pressure.get())


    # get next Matrix

    # get next action

    # do next action

main()