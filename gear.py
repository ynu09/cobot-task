import rclpy
import time
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_digital_input,
            set_digital_output,
            amove_periodic,
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            wait,
            mwait,
            check_force_condition,
            check_position_condition,
            task_compliance_ctrl,
            set_desired_force,
            release_compliance_ctrl,
            DR_BASE,
            DR_TOOL,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
        )
        from DR_common2 import posx
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
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
        print("grip")
    
    def detecting():
        # Compliance
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # Force
        set_desired_force(
            fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL
        )
        
        while True:
            time.sleep(0.5)
            if check_force_condition(axis=DR_AXIS_Z, max=5):
                print("Force condition met")
                amove_periodic(amp=[0, 0, 0, 0, 0, 9], period=1.5, atime=1.00, repeat=3, ref=DR_TOOL)
                
                while True:
                    if  check_position_condition(axis=DR_AXIS_Z, min= 278, max= 281, ref=DR_BASE):
                        print("position condition met")
                        break
                break          
        release_compliance_ctrl()
        mwait()
    
    def center_gear():
        
        left_top = posx([425.00579833984375, -0.2594680190086365, 337.7929382324219, 138.79440307617188, -179.8468475341797, 138.66094970703125]) 
        left_bottom = list(trans(left_top, [0.0, 0.0, -66.62164306640625, 0.0, 0.0, 0.0], DR_BASE))
        right_top = list(trans(left_top, [247.712646484375, 0.0, 0.0, 0.0, 0.0, 0.0], DR_BASE))
        right_bottom = list(trans(right_top, [0.0, 0.0, -45, 0.0, 0.0, 0.0], DR_BASE))

        release()

        print("move start")
        movel(left_top, vel=VELOCITY, acc=ACC)
        movel(left_bottom, vel=VELOCITY, acc=ACC)
        grip()
        movel(left_top, vel=VELOCITY, acc=ACC)
        movel(right_top, vel=VELOCITY, acc=ACC)
        movel(right_bottom, vel=VELOCITY, acc=ACC)
        detecting()
        release()
        movel(right_top, vel=VELOCITY, acc=ACC)
        print("move finish")


    set_tool("Tool Weight_RG2")
    set_tcp("RG2_TCP")

    JReady = [0, 0, 90, 0, 90, 0]

    if rclpy.ok():
        print("home")
        movej(JReady, vel=VELOCITY, acc=ACC)
        center_gear()

    rclpy.shutdown()


if __name__ == "__main__":
    main()