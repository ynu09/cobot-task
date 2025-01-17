import rclpy

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sports_stacking", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_TOOL,
            get_current_posx,
            get_digital_input,
            movej,
            movel,
            mwait,
            set_digital_output,
            set_tcp,
            set_tool,
            trans,
            wait,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
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

    # 컵 뭉치를 이동
    def move_whole_cups(pos_start, pos_goal):
        pos_start[2] = 5.5
        pos_goal[2] = 5.5
        movel(trans(pos_start, pos_up, ref=DR_BASE), VELOCITY, ACC)
        movel(pos_start, VELOCITY, ACC)

        movel(pos_goal, VELOCITY, ACC)

    # 각 층의 삼각형 모양 컵 좌표 리스트를 만드는 함수
    def make_positions_floor(floor=1, ref=[0, 0, 0, 0, 0, 0]):
        size_cup = {"height": 95, "gap": 11, "diameter": 75 * 1}
        triangle_height = 64.95 * 1.1

        ret_poses = []

        for i in range(3 - (floor - 1), 0, -1):
            for j in range(i):
                x = float(
                    j * size_cup["diameter"]
                    + (3 - (floor - 1) - i - 1)
                    * (triangle_height * (1 / 3))
                    * (floor - 1)
                    - (size_cup["diameter"] / 2 * i)
                    + (floor - 1) * (size_cup["diameter"] / 8)
                )
                y = float(
                    (3 - (floor - 1) - i) * triangle_height
                    + (triangle_height * (1 / 3)) * (floor - 1)
                )
                # 인자로 받은 ref 좌표를 기준으로하고, 계산한 x, y만큼 이동한 좌표를 리스트에 저장
                ret_poses.append(list(trans(ref, [x, y, 0, 0, 0, 0], DR_BASE)))

        ret_poses = sorted(ret_poses, key=lambda x: x[1], reverse=True)
        return ret_poses

    # 컵 뭉치 중 원하는 개수를 시작좌표에서 목표좌표로 이동
    def split_cups(pos_start, pos_goal, total_num, pick_num):
        if total_num < pick_num:
            raise ValueError("total_num should be greater than pick_num")

        # 시작 좌표와 목표좌표에서 컵뭉치를 잡을 그리퍼 포즈
        pos_start[3:6] = [90.43, 103.756, -90.889]
        pos_goal[3:6] = [90.43, 103.756, -90.889]

        # 컵 뭉치의 최대 높이
        pos_grappoint_max = list(
            trans(pos_start, [0, 0, size_cup["gap"] * total_num, 0, 0, 0], DR_BASE)
        )

        # 컵 뭉치 중 선택한 개수를 잡을 그리퍼 높이
        height_grappoint = float(
            -pick_num * size_cup["gap"]
            - int(bool(total_num - pick_num)) * size_cup["gap"] / 4
        )

        # 컵 개수를 통해 계산한 그리퍼 포즈
        pos_grappoint = trans(
            pos_grappoint_max, [0, 0, height_grappoint, 0, 0, 0], DR_BASE
        )
        release()
        movel(pos_grappoint, VELOCITY, ACC)

        # 그리퍼를 두번 잡아 컵이 떨어질 확률을 높임
        grip()
        release()
        grip()

        movel(trans(pos_grappoint_max, pos_up, ref=DR_BASE), VELOCITY, ACC)

        movel(trans(list(pos_goal), pos_up, ref=DR_BASE), VELOCITY, ACC)

        movel(pos_goal, VELOCITY, ACC)
        release()
        mwait()

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 홈 포즈
    JReady = [-45, 0, 90, 0, 90, 0]
    # 초기 컵 뭉치 위치
    Lcup = [575.234, -0.132, 5.5, 90.43, 103.756, -90.889]

    # 컵 크기 측정치
    size_cup = {"height": 95, "gap": 11, "diameter": 75}
    # 그리퍼로 잡고 올릴 때 사용하는 상대 포즈 상수
    pos_up = [0, 0, size_cup["height"] + 20, 0, 0, 0]

    while rclpy.ok():
        movel(trans(get_current_posx()[0], [0, 0, 100, 0, 0, 0]), VELOCITY, ACC)

        print("home")
        movej(JReady, VELOCITY, ACC)
        release()

        pos_last = list(trans(Lcup, [0, 0, 0, 0, 0, 0], DR_BASE))

        # 1층 컵 좌표 리스트
        poses_first_floor = make_positions_floor(
            1, list(trans(pos_last, [0, 0, 0, 0, 0, 0], DR_BASE))
        )

        # 1층 코드
        cups_num_stacked = 11
        for goal in poses_first_floor:
            # 2 - 11 번째
            if pos_last in poses_first_floor:
                cups_num_to_move = cups_num_stacked - 1
                split_cups(pos_last, goal, cups_num_stacked, cups_num_to_move)
                cups_num_stacked -= 1
            # 1번째
            else:
                cups_num_to_move = cups_num_stacked
                move_whole_cups(pos_last, goal)

            pos_last = goal

        # 2층 코드
        ref = Lcup
        ref[2] = 5.5 + 95  # 2층 z좌표
        poses_second_floor = make_positions_floor(
            2, list(trans(ref, [0, -5, 0, 0, 0, 0], DR_BASE))
        )
        for goal in poses_second_floor:
            cups_num_to_move = cups_num_stacked - 1
            split_cups(pos_last, goal, cups_num_stacked, cups_num_to_move)
            cups_num_stacked -= 1
            pos_last = goal

        # 3층 코드
        ref = Lcup
        ref[2] = 5.5 + 95 + 95  # 3층 z좌표
        poses_third_floor = make_positions_floor(
            3, list(trans(ref, [0, 0, 0, 0, 0, 0], DR_BASE))
        )
        for goal in poses_third_floor:
            cups_num_to_move = cups_num_stacked - 1
            split_cups(pos_last, goal, cups_num_stacked, cups_num_to_move)
            cups_num_stacked -= 1
            pos_last = goal

        movel(trans(pos_last, pos_up, ref=DR_BASE), VELOCITY, ACC)
        movel(pos_last, VELOCITY, ACC)
        grip()

        # 4층 코드 별도 작성
        # 4층 컵을 뒤집기 위해 그리퍼를 수평으로 만들기 위한 rx, ry, rz
        pos_last[3:6] = [90, 90, -90]
        movel(trans(pos_last, [0, 0, 200, 0, 0, 0], ref=DR_BASE), VELOCITY, ACC)
        movel(
            trans(get_current_posx()[0], [0, 0, 0, 0, 0, 180], ref=DR_TOOL),
            VELOCITY,
            ACC,
        )
        movel(
            trans(get_current_posx()[0], [0, 0, -50, 0, 0, 0], ref=DR_BASE),
            VELOCITY,
            ACC,
        )
        release()
        movel(
            trans(get_current_posx()[0], [0, -100, 0, 0, 0, 0], ref=DR_BASE),
            VELOCITY,
            ACC,
        )

        # 마무리 포즈
        movej([0, 0, 0, 0, 0, 0], VELOCITY, ACC)

        break

    rclpy.shutdown()


if __name__ == "__main__":
    main()
