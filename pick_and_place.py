import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_and_place", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            wait,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)
        print("release")

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
        print("grip")
    
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([399.20, -47.82, 135.0, 76.59, -179.79, 75.86])
    pos2 = list(trans(pos1, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos3 = list(trans(pos2, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos4 = list(trans(pos1, [0.0, -52.91, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos5 = list(trans(pos4, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos6 = list(trans(pos5, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos7 = list(trans(pos4, [0.0, -52.91, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos8 = list(trans(pos7, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
    pos9 = list(trans(pos8, [50.76, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE)) 

    def move(pos):

        down1 = list(trans(pos, [0.0, 0.0, -100.0, 0.0, 0.0, 0.0], DR_BASE))
        up = list(trans(down1, [0.0, 0.0, 100.0, 0.0, 0.0, 0.0], DR_BASE))
        move = list(trans(up, [0.0, 147.74, 0.0, 0.0, 0.0, 0.0], DR_BASE))
        down2 = list(trans(move, [0.0, 0.0, -90.0, 0.0, 0.0, 0.0], DR_BASE))
        

        release()
        
        print("move start")
        movel(pos, vel=VELOCITY, acc=ACC)
        movel(down1, vel=VELOCITY, acc=ACC)
        grip()
        movel(up, vel=VELOCITY, acc=ACC)
        movel(move, vel=VELOCITY, acc=ACC)
        movel(down2, vel=VELOCITY, acc=ACC)
        release()
        print("move finish")

        
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        print("home")
        movej(JReady, vel=VELOCITY, acc=ACC)
        move(pos1)
        move(pos2)
        move(pos3)
        move(pos4)
        move(pos5)
        move(pos6)
        move(pos7)
        move(pos8)
        move(pos9)
        break

    rclpy.shutdown()

if __name__ == "__main__":
    main()