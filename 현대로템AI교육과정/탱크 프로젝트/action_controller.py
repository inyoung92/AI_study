import time
import queue
import math
import logging
import copy 
import json # JSON 직렬화를 위해 임포트

from core.a_star_pathfinder import AStarPathfinder
from core.utils import clear_queue 

def normalize_angle(angle):
    """각도를 -180도 ~ 180도 범위로 정규화"""
    angle = math.fmod(angle + 180, 360)
    if angle < 0:
        angle += 360
    return angle - 180

def simplify_path(path, tolerance=5.0):
    """
    주어진 경로를 단순화하여 웨이포인트 수를 줄입니다.
    `tolerance`는 경로가 직선에서 벗어나는 허용치를 나타냅니다.
    값이 클수록 더 많이 단순화됩니다.
    """
    if len(path) < 3:
        return path

    simplified_path = [path[0]] # 시작점은 항상 포함

    start_index = 0
    end_index = 0

    while end_index < len(path) - 1:
        p1 = path[start_index]
        p2_candidate_index = start_index + 1
        
        furthest_point_in_segment_index = -1
        max_deviation_in_segment = -1.0

        for i in range(start_index + 1, len(path)):
            current_point = path[i]
            p2_candidate = current_point 
            
            current_max_dev = -1.0
            current_furthest_idx = -1
            
            for j in range(start_index + 1, i + 1): # start_index 다음부터 current_point까지
                temp_point = path[j]
                
                numerator = abs((p2_candidate['x'] - p1['x']) * (p1['z'] - temp_point['z']) - \
                                (p1['x'] - temp_point['x']) * (p2_candidate['z'] - p1['z']))
                denominator = math.sqrt((p2_candidate['x'] - p1['x'])**2 + (p2_candidate['z'] - p1['z'])**2)
                
                distance = 0
                if denominator != 0:
                    distance = numerator / denominator 
                
                if distance > current_max_dev:
                    current_max_dev = distance
                    current_furthest_idx = j

            if current_max_dev > tolerance:
                simplified_path.append(path[furthest_point_in_segment_index])
                start_index = furthest_point_in_segment_index
                end_index = i 
                break 
            else:
                if current_max_dev > max_deviation_in_segment:
                    max_deviation_in_segment = current_max_dev
                    furthest_point_in_segment_index = current_furthest_idx

                end_index = i 

        else: 
            simplified_path.append(path[len(path) - 1])
            break

    if path and path[-1] not in simplified_path:
        simplified_path.append(path[-1])

    return simplified_path


def simplify_path(path, tolerance=5.0):
    # ... (기존 simplify_path 함수는 그대로 유지) ...
    if len(path) < 3:
        return path

    simplified_path = [path[0]]

    start_index = 0
    end_index = 0

    while end_index < len(path) - 1:
        p1 = path[start_index]
        p2_candidate_index = start_index + 1
        
        furthest_point_in_segment_index = -1
        max_deviation_in_segment = -1.0

        for i in range(start_index + 1, len(path)):
            current_point = path[i]
            p2_candidate = current_point 
            
            current_max_dev = -1.0
            current_furthest_idx = -1
            
            for j in range(start_index + 1, i + 1):
                temp_point = path[j]
                
                numerator = abs((p2_candidate['x'] - p1['x']) * (p1['z'] - temp_point['z']) - \
                                (p1['x'] - temp_point['x']) * (p2_candidate['z'] - p1['z']))
                denominator = math.sqrt((p2_candidate['x'] - p1['x'])**2 + (p2_candidate['z'] - p1['z'])**2)
                
                distance = 0
                if denominator != 0:
                    distance = numerator / denominator 
                
                if distance > current_max_dev:
                    current_max_dev = distance
                    current_furthest_idx = j

            if current_max_dev > tolerance:
                simplified_path.append(path[furthest_point_in_segment_index])
                start_index = furthest_point_in_segment_index
                end_index = i 
                break 
            else:
                if current_max_dev > max_deviation_in_segment:
                    max_deviation_in_segment = current_max_dev
                    furthest_point_in_segment_index = current_furthest_idx

                end_index = i 

        else: 
            simplified_path.append(path[len(path) - 1])
            break

    if path and path[-1] not in simplified_path:
        simplified_path.append(path[-1])

    return simplified_path


def action_worker(action_input_q, action_output_q, hit_input_q, detect_input_q,
                  info_input_q, info_output_q, init_input_q, collision_input_q,
                  path_request_q, path_result_q, obstacle_q):
    
    # --- action_worker 프로세스 내부 로깅 설정 ---
    worker_log_file_name = 'tank_simulation_action_worker.log' 
    map_data_log_file_name = 'map_data.log' 
    
    logger = logging.getLogger('action_worker_process') 
    logger.setLevel(logging.DEBUG)
    if logger.hasHandlers():
        logger.handlers.clear()

    file_handler = logging.FileHandler(worker_log_file_name, mode='a', encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(processName)s - %(message)s'))
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.INFO)
    stream_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(processName)s - %(message)s'))
    logger.addHandler(stream_handler)
    logger.info("🚀 전차 목표 추적 시스템 (Action Worker) 시작")

    # --- 맵 데이터 전용 로거 설정 ---
    map_logger = logging.getLogger('map_data_logger')
    map_logger.setLevel(logging.INFO) 
    if map_logger.hasHandlers():
        map_logger.handlers.clear()
    
    map_file_handler = logging.FileHandler(map_data_log_file_name, mode='w', encoding='utf-8') 
    map_file_handler.setLevel(logging.INFO)
    map_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s')) 
    map_logger.addHandler(map_file_handler)
    map_logger.info("--- Map Data Log Started ---")

    global pathfinder 
    pathfinder = None 
    
    target_x = None
    target_z = None
    is_moving = False
    
    current_x = None 
    current_z = None
    current_yaw = None

    current_speed = 0.0 
    acceleration_rate = 0.05 
    deceleration_rate = 0.08 
    
    max_forward_speed = 1.0 
    max_forward_speed_decel_phase = 0.6 

    early_decel_distance = 40.0 
    
    waypoint_arrival_radius = 10.0 
    
    angle_threshold = 5.0 
    min_rotation_weight = 0.1 
    max_rotation_weight = 0.8 
    
    while True:
        # 1. 초기화 처리 (main에서 받은 config 데이터로 A* Pathfinder 초기화)
        try:
            init_data = init_input_q.get_nowait() 
            map_info = init_data.get("map_info")

            if map_info:
                map_width = map_info.get("map_width")
                map_height = map_info.get("map_height")
                grid_size = map_info.get("grid_size")
                
                initial_raw_obstacles = map_info.get("initial_obstacles", [])
                
                # 초기 장애물 데이터 변환 로직
                initial_obstacles = []
                for obs_raw in initial_raw_obstacles:
                    if all(k in obs_raw for k in ['x_min', 'x_max', 'z_min', 'z_max']):
                        center_x = (obs_raw['x_min'] + obs_raw['x_max']) / 2.0
                        center_z = (obs_raw['z_min'] + obs_raw['z_max']) / 2.0
                        radius = max(abs(obs_raw['x_max'] - obs_raw['x_min']), 
                                     abs(obs_raw['z_max'] - obs_raw['z_min'])) / 2.0 + 1.0 
                        initial_obstacles.append({'x': center_x, 'z': center_z, 'radius': radius})
                    else:
                        logger.warning(f"⚠️ Initial obstacle data invalid format: {obs_raw}. Skipping.")

                pathfinder = AStarPathfinder(map_width, map_height, grid_size)
                pathfinder.update_obstacles(initial_obstacles) 
                logger.info(f"A* Pathfinder 초기화 완료 (맵: {map_width}x{map_height}, 그리드: {grid_size})")
                
                map_logger.info(f"Map Initialized: Width={map_width}, Height={map_height}, GridSize={grid_size}")
                if initial_obstacles:
                    map_logger.info(f"Processed Initial Obstacles: {json.dumps(initial_obstacles)}")
                
                # --- 초기화 시점 그리드 시각화 (선택 사항) ---
                # pathfinder.visualize_grid(obstacles_only=True) # 초기 맵만 보고 싶을 때
                # ---------------------------------------------

            logger.info("🛠️ 시뮬레이션 초기화 (Action Worker)")
            target_x = None
            target_z = None
            is_moving = False
            current_speed = 0.0 
            info_output_q.put({"status": "initialized"}) 
            continue
        except queue.Empty:
            pass
        
        # 2. 장애물 정보 업데이트 수신 및 A* Pathfinder에 전달
        try:
            obstacle_data = obstacle_q.get_nowait()
            if pathfinder:
                raw_obstacles = obstacle_data.get("obstacles", [])
                
                # 동적으로 수신된 장애물 데이터 변환 로직
                processed_obstacles = []
                for obs_raw in raw_obstacles:
                    if all(k in obs_raw for k in ['x_min', 'x_max', 'z_min', 'z_max']):
                        center_x = (obs_raw['x_min'] + obs_raw['x_max']) / 2.0
                        center_z = (obs_raw['z_min'] + obs_raw['z_max']) / 2.0
                        radius = max(abs(obs_raw['x_max'] - obs_raw['x_min']), 
                                     abs(obs_raw['z_max'] - obs_raw['z_min'])) / 2.0 + 1.0
                        processed_obstacles.append({'x': center_x, 'z': center_z, 'radius': radius})
                    else:
                        logger.warning(f"⚠️ Dynamic obstacle data invalid format: {obs_raw}. Skipping.")

                pathfinder.update_obstacles(processed_obstacles) 
                logger.info(f"A* Pathfinder에 장애물 정보 업데이트 요청 수신. 현재 장애물 수: {len(processed_obstacles)}")
                if processed_obstacles:
                    map_logger.info(f"Dynamic Obstacle Update: {json.dumps(processed_obstacles)}")
            else:
                logger.warning("A* Pathfinder가 초기화되지 않아 장애물 정보 업데이트 불가.")
        except queue.Empty:
            pass

        # 3. 경로 탐색 요청 수신 (main으로부터)
        try:
            path_req = path_request_q.get_nowait()
            if path_req.get("request_type") == "find_path":
                if current_x is not None and current_z is not None and pathfinder:
                    target_pos = path_req.get("target_pos")
                    
                    start_world = (current_x, current_z)
                    end_world = (target_pos["x"], target_pos["z"])

                    raw_path = pathfinder.find_path(start_world, end_world)
                    
                    simplified_path = simplify_path(raw_path, tolerance=10.0) 
                    logger.info(f"경로 단순화 적용: 원본 길이 {len(raw_path)}, 단순화 후 길이 {len(simplified_path)}")
                    
                    path_result_q.put({"status": "path_calculated", "path": simplified_path})
                    info_output_q.put({"status": "path_calculated", "path": simplified_path}) 
                    logger.info(f"A* 경로 계산 및 단순화 완료, main으로 전송. 최종 길이: {len(simplified_path)}")
                    
                    # --- 경로 계산 후 그리드 시각화 호출 ---
                    if simplified_path: # 단순화된 경로가 존재할 때만 시각화
                        pathfinder.visualize_grid(path=simplified_path, start_pos=start_world, end_pos=end_world)
                    else:
                        logger.warning("경로를 찾을 수 없어 시각화할 수 없습니다.")
                        # 경로를 찾지 못했을 때 그리드만 시각화하고 싶다면:
                        # pathfinder.visualize_grid(start_pos=start_world, end_pos=end_world)
                    # ------------------------------------

                else:
                    logger.warning("경로 탐색 요청 수신, 그러나 현재 전차 위치 또는 pathfinder 초기화 안됨.")
                    info_output_q.put({"status": "path_calculated", "path": []}) 
        except queue.Empty:
            pass

        # 4. 목표 좌표 수신 (set_destination 또는 MainProcess의 load_next_waypoint)
        try:
            destination_data = action_input_q.get_nowait()
            if isinstance(destination_data, dict) and "destination" in destination_data:
                dest = destination_data["destination"]
                if dest == "STOP": 
                    target_x = None
                    target_z = None
                    is_moving = False
                    current_speed = 0.0 
                    logger.info("✅ 모든 웨이포인트 이동 완료 신호 수신. 전차 정지.")
                    
                    clear_queue(action_output_q) 
                    
                    continue 
                else:
                    target_x = dest["x"]
                    target_z = dest["z"]
                    is_moving = True
                    logger.info(f"🎯 새로운 목표 설정: ({target_x:.2f}, {target_z:.2f})")
        except queue.Empty:
            pass

        # 5. 현재 상태 정보 수신: 가장 최신 정보만 가져오도록 변경
        latest_log_data = None
        while not info_input_q.empty():
            try:
                latest_log_data = info_input_q.get_nowait()
            except queue.Empty:
                break 
        
        if not latest_log_data:
            time.sleep(0.001) 
            continue 

        # 데이터 파싱
        current_x = latest_log_data.get("playerPos", {}).get("x", 0.0)
        current_z = latest_log_data.get("playerPos", {}).get("z", 0.0)
        current_yaw = normalize_angle(latest_log_data.get("stereoCameraLeftRot", {}).get("y", 0.0))
        
        log_data_for_debug = copy.deepcopy(latest_log_data)
        if "lidarPoints" in log_data_for_debug:
            del log_data_for_debug["lidarPoints"]
        logger.debug(f"Received latest /info data (lidarPoints excluded): {log_data_for_debug}")
        
        action = {
            "moveWS": {"command": "", "weight": 0.0},
            "moveAD": {"command": "", "weight": 0.0},
            "turretQE": {"command": "", "weight": 0.0},
            "turretRF": {"command": "", "weight": 0.0},
            "fire": False
        }
        
        if target_x is None or target_z is None:
            current_speed = 0.0
            time.sleep(0.001) 
            continue 
        
        dx = target_x - current_x
        dz = target_z - current_z
        distance_to_target = math.sqrt(dx*dx + dz*dz)
        
        target_angle = math.degrees(math.atan2(dx, dz))
        target_angle = normalize_angle(target_angle)
        
        angle_diff = normalize_angle(target_angle - current_yaw) 
        
        logger.debug(f"위치: ({current_x:.1f}, {current_z:.1f}) → 목표: ({target_x:.1f}, {target_z:.1f})")
        logger.debug(f"거리: {distance_to_target:.1f}m, 각도차이: {angle_diff:.1f}°")
        
        # --- 웨이포인트 도착 판단 및 다음 목표 요청 ---
        if distance_to_target < waypoint_arrival_radius:
            logger.info(f"✨ 웨이포인트 도달 반경 내 진입! ({waypoint_arrival_radius:.1f}m)")
            info_output_q.put({"status": "arrived_at_waypoint"}) 
            
            target_x = None 
            target_z = None
            is_moving = False
            current_speed = 0.0 
            time.sleep(0.001)
            continue 
        
        # 각도 제어
        strict_angle_threshold_for_turn = 1.0 
        
        if abs(angle_diff) > strict_angle_threshold_for_turn: 
            rotation_scale = min(abs(angle_diff) / 90.0, 1.0)
            rotation_weight = min_rotation_weight + (max_rotation_weight - min_rotation_weight) * rotation_scale
            
            if abs(angle_diff) < 15: 
                rotation_weight = max(min_rotation_weight, rotation_weight * (abs(angle_diff) / 15.0))
            
            if angle_diff > 0:
                action["moveAD"]["command"] = "D"
            else:
                action["moveAD"]["command"] = "A"
            
            action["moveAD"]["weight"] = rotation_weight
            logger.info(f"🔄 회전: {angle_diff:.1f}° (가중치: {rotation_weight:.2f})")
        else: 
            action["moveAD"]["command"] = ""
            action["moveAD"]["weight"] = 0.0
            logger.debug("정렬 완료 (회전 없음)")

        # 전진 제어 로직
        can_move_forward = abs(angle_diff) < 15.0 

        action["moveWS"]["command"] = ""
        action["moveWS"]["weight"] = 0.0

        effective_decel_start_distance = early_decel_distance 

        if can_move_forward and distance_to_target > waypoint_arrival_radius: 
            if distance_to_target > effective_decel_start_distance:
                current_speed = min(current_speed + acceleration_rate, max_forward_speed)
                action["moveWS"]["command"] = "W"
                action["moveWS"]["weight"] = current_speed
                logger.info(f"➡️ 전진: 속도 {current_speed:.2f} (거리: {distance_to_target:.1f}m)")
                
            else: 
                normalized_distance = (distance_to_target - waypoint_arrival_radius) / (effective_decel_start_distance - waypoint_arrival_radius)
                target_speed_based_on_distance = max(0.0, min(1.0, normalized_distance)) * max_forward_speed_decel_phase
                
                if current_speed > target_speed_based_on_distance + 0.02: 
                    current_speed = max(0.0, current_speed - deceleration_rate * 2.5) 
                elif current_speed < target_speed_based_on_distance - 0.02: 
                    current_speed = min(max_forward_speed_decel_phase, current_speed + acceleration_rate * 0.5) 
                
                action["moveWS"]["command"] = "W"
                action["moveWS"]["weight"] = current_speed
                logger.info(f"🛑 웨이포인트 감속 단계: 속도 {current_speed:.2f} (거리: {distance_to_target:.1f}m, 목표 속도: {target_speed_based_on_distance:.2f})")
                
        else: 
            if current_speed > 0.01: 
                current_speed = max(0.0, current_speed - deceleration_rate * 4) 
                action["moveWS"]["command"] = "W"
                action["moveWS"]["weight"] = current_speed
                logger.info(f"🚨 긴급 감속/정지: 속도 {current_speed:.2f}")
            else:
                current_speed = 0.0
                action["moveWS"]["command"] = ""
                action["moveWS"]["weight"] = 0.0
                logger.info(f"🚫 완전 중립 정지 (거리: {distance_to_target:.1f}m)")
        
        # 기타 입력 처리 (논블로킹)
        try:
            detections = detect_input_q.get_nowait()
            # 여기에 감지된 내용 처리 로직 추가 (예: 터렛 회전, 발사 명령 등)
            # logger.debug(f"Received detections: {detections}") # 너무 많은 로그 방지
        except queue.Empty:
            pass
        
        try:
            hit_data = hit_input_q.get_nowait()
            logger.info(f"💥 포탄 충돌 감지: {hit_data}")
        except queue.Empty:
            pass
        
        try:
            collision_data = collision_input_q.get_nowait()
            logger.info(f"🚨 충돌 감지: {collision_data}")
            # 충돌 발생 시 비상 정지 또는 회피 로직 추가 고려
        except queue.Empty:
            pass
        
        # 액션 전송: 항상 최신 액션만 전송
        clear_queue(action_output_q) 
        action_output_q.put(action) 
        
        time.sleep(0.001)
