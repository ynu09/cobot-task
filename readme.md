## 두산 협동로봇 M0609 모델 동작 운영 실습

### 기록

| 기간 | 내용 | 파일 |
| --- | --- | --- |
| 2024.12.24 ~ 2024.12.31 | Doosan Robot ROS 패키지 사용(https://github.com/Juwan-s/doosan-robot2), ROS2 프로그램을 활용한 협동로봇 동작 운영 실습(https://manual.doosanrobotics.com/ko/programming/2.11.2/Publish/ 참고하여 코드 작성) | pick_and_place.py, gear.py, gripper_setting.json, sports_stacking.py |

### 동작 종류

1. Pick and Place
    - 주요 함수
        
        
        | 함수 | 설명 |
        | --- | --- |
        | trans | ref 좌표계 기준으로 정의된 pos(포즈)를 동일 좌표계를 기준으로 delta만큼 이동/회전하여 ref_out 좌표계 기준으로 변환한 후 리턴 |
2. Gear 조립
    - 주요 함수
        
        
        | 함수 | 설명 |
        | --- | --- |
        | set_desired_force | 전역으로 설정된 좌표계(set_ref_coord() 참조) 기준으로 힘 제어 목표값, 힘 제어 방향, 힘 목표값, 외력참조모드 설정 |
        | amove_periodic | 비동기(async.)방식의 move_periodic모션 |
        | check_position_condition | 주어진 위치 상태를 확인 (기어가 원하는 위치까지 내려왔는지) |
3. Sports Stacking
    - gripper의 정밀한 설정
        
        ![img1](https://github.com/user-attachments/assets/37e685b4-5217-4111-9653-f61098c0c052)
        
    - 바닥 위치 계산
        
        ![img2](https://github.com/user-attachments/assets/96d04119-d1a7-4358-b31c-77942a8feca9)
        
    - 주요 함수
        
        
        | 함수 | 설명 |
        | --- | --- |
        | sorted() | 컵을 위쪽이 아닌 가로방향으로 잡기 때문에 그리퍼와의 충돌로 나중에 놓을 수 없는 자리 발생 → sorted를 사용해 Y축 좌표가 큰 곳부터 컵을 배치 |
        | make_positions_ﬂoor(ﬂoor=1, ref=[0, 0, 0, 0, 0, 0]) → list(posx) | 층 수와 기준좌표를 인자로 받아 층 별 필요한 삼각형 좌표에 기준좌표 ref 를 trans한 좌표 리스트를 반환 |
        | split_cups(pos_start, pos_goal, total_num, pick_num) | 시작좌표에 있는 컵 뭉치 중 원하는 개수를 집어 목표좌표로 이동 |

### 기술 스택

| 분류 | 기술 |
| --- | --- |
| 개발 환경 | Ubuntu 22.04 |
| 개발 언어 | Python |
| 통신 프로토콜 | ROS2 |

### 결과물

[View PDF Document](./그룹E-2_8주차_협동-2_산출물.pdf)

### 실행 영상

[![Pick and Place](https://img.youtube.com/vi/2L6M99BnuI4/hqdefault.jpg)](https://www.youtube.com/watch?v=2L6M99BnuI4)
[![Gear](https://img.youtube.com/vi/ZicDkgHe5TU/hqdefault.jpg)](https://www.youtube.com/watch?v=ZicDkgHe5TU)
[![Sports Stacking](https://img.youtube.com/vi/-iTpF9ZZGLU/hqdefault.jpg)](https://www.youtube.com/watch?v=-iTpF9ZZGLU)
