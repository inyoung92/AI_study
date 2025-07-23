from flask import Flask, request, jsonify
import os
import torch
from ultralytics import YOLO

app = Flask(__name__)
model = YOLO('yolov8n.pt')

combined_commands = [
    {
        "moveWS": {"command": "W", "weight": 1.0},
        "moveAD": {"command": "D", "weight": 1.0},
        "turretQE": {"command": "Q", "weight": 0.7},
        "turretRF": {"command": "R", "weight": 0.5},
        "fire": False
    },
    {
        "moveWS": {"command": "W", "weight": 0.6},
        "moveAD": {"command": "A", "weight": 0.4},
        "turretQE": {"command": "E", "weight": 0.8},
        "turretRF": {"command": "R", "weight": 0.3},
        "fire": True
    },
    {
        "moveWS": {"command": "W", "weight": 0.5},
        "moveAD": {"command": "", "weight": 0.0},
        "turretQE": {"command": "E", "weight": 0.4},
        "turretRF": {"command": "R", "weight": 0.6},
        "fire": False
    },
    {
        "moveWS": {"command": "W", "weight": 0.3},
        "moveAD": {"command": "D", "weight": 0.3},
        "turretQE": {"command": "E", "weight": 0.5},
        "turretRF": {"command": "R", "weight": 0.7},
        "fire": True
    },
    {
        "moveWS": {"command": "W", "weight": 1.0},
        "moveAD": {"command": "", "weight": 0.0},
        "turretQE": {"command": "E", "weight": 0.5},
        "turretRF": {"command": "R", "weight": 0.5},
        "fire": False
    },
    {
        "moveWS": {"command": "W", "weight": 0.8},
        "moveAD": {"command": "A", "weight": 0.6},
        "turretQE": {"command": "E", "weight": 0.9},
        "turretRF": {"command": "R", "weight": 0.2},
        "fire": True
    },
    {
        "moveWS": {"command": "W", "weight": 1.0},
        "moveAD": {"command": "D", "weight": 1.0},
        "turretQE": {"command": "E", "weight": 1.0},
        "turretRF": {"command": "R", "weight": 1.0},
        "fire": True
    },
    {
        "moveWS": {"command": "W", "weight": 0.2},
        "moveAD": {"command": "A", "weight": 0.9},
        "turretQE": {"command": "", "weight": 0.0},
        "turretRF": {"command": "R", "weight": 0.9},
        "fire": False
    },
    {
        "moveWS": {"command": "S", "weight": 0.4},
        "moveAD": {"command": "D", "weight": 0.4},
        "turretQE": {"command": "E", "weight": 0.6},
        "turretRF": {"command": "F", "weight": 0.6},
        "fire": True
    },
    {
        "moveWS": {"command": "W", "weight": 0.8},
        "moveAD": {"command": "", "weight": 0.0},
        "turretQE": {"command": "Q", "weight": 0.5},
        "turretRF": {"command": "", "weight": 0.0},
        "fire": False
    },
    {
        "moveWS": {"command": "STOP", "weight": 1.0},
        "moveAD": {"command": "", "weight": 0.0},
        "turretQE": {"command": "", "weight": 0.0},
        "turretRF": {"command": "", "weight": 0.0},
        "fire": True
    },
    {
        "moveWS": {"command": "S", "weight": 0.2},
        "moveAD": {"command": "A", "weight": 0.2},
        "turretQE": {"command": "E", "weight": 0.2},
        "turretRF": {"command": "F", "weight": 0.2},
        "fire": False
    }
]


@app.route('/detect', methods=['POST'])
def detect():
    image = request.files.get('image')
    if not image:
        return jsonify({"error": "No image received"}), 400

    image_path = 'temp_image.jpg'
    image.save(image_path)

    results = model(image_path)
    detections = results[0].boxes.data.cpu().numpy()

    target_classes = {0: "person", 2: "car", 7: "truck", 15: "rock"}
    filtered_results = []
    for box in detections:
        class_id = int(box[5])
        if class_id in target_classes:
            filtered_results.append({
                'className': target_classes[class_id],
                'bbox': [float(coord) for coord in box[:4]], # Bound Box의 좌표 [ x1, y1, x2, y2]
                'confidence': float(box[4]), # 신뢰도
                'color': '#00FF00',
                'filled': False,
                'updateBoxWhileMoving': True # 전차 이동시 Bound Box보정 기능 사용 유무
            })

    return jsonify(filtered_results)

@app.route('/info', methods=['POST']) 
# 시뮬레이터에서 전송된 로그데이터를 수신
def info():
    data = request.get_json(force=True)
    if not data:
        return jsonify({"error": "No JSON received"}), 400

    #print("📨 /info data received:", data)

    # Auto-pause after 15 seconds
    #if data.get("time", 0) > 15:
    #    return jsonify({"status": "success", "control": "pause"})
    # Auto-reset after 15 seconds
    #if data.get("time", 0) > 15:
    #    return jsonify({"stsaatus": "success", "control": "reset"})
    return jsonify({"status": "success", "control": ""})

@app.route('/get_action', methods=['POST'])
# 전차의 현재 위치와 포탑의 정보를 취득하고 이동과 포탑 제어를 위한 다음 액션 명령을 제공
def get_action():
    data = request.get_json(force=True)

    position = data.get("position", {})
    turret = data.get("turret", {})

    pos_x = position.get("x", 0)
    pos_y = position.get("y", 0)
    pos_z = position.get("z", 0)

    turret_x = turret.get("x", 0)
    turret_y = turret.get("y", 0)

    print(f"📨 Position received: x={pos_x}, y={pos_y}, z={pos_z}")
    print(f"🎯 Turret received: x={turret_x}, y={turret_y}")

    if combined_commands:
        command = combined_commands.pop(0)
    else:
        command = {
            "moveWS": {"command": "STOP", "weight": 1.0},
            "moveAD": {"command": "", "weight": 0.0},
            "turretQE": {"command": "", "weight": 0.0},
            "turretRF": {"command": "", "weight": 0.0},
            "fire": False
        }

    print("🔁 Sent Combined Action:", command)
    return jsonify(command)

@app.route('/update_bullet', methods=['POST'])
# 포탄이 충돌한 위치 및 대상 정보를 End Point에 전달
def update_bullet():
    data = request.get_json()
    if not data:
        return jsonify({"status": "ERROR", "message": "Invalid request data"}), 400

    print(f"💥 Bullet Impact at X={data.get('x')}, Y={data.get('y')}, Z={data.get('z')}, Target={data.get('hit')}")
    return jsonify({"status": "OK", "message": "Bullet impact data received"})

@app.route('/set_destination', methods=['POST'])
# Tracking Edit Mode 에서 설정한 목적지를 전달
def set_destination():
    data = request.get_json()
    if not data or "destination" not in data:
        return jsonify({"status": "ERROR", "message": "Missing destination data"}), 400

    try:
        x, y, z = map(float, data["destination"].split(","))
        print(f"🎯 Destination set to: x={x}, y={y}, z={z}")
        return jsonify({"status": "OK", "destination": {"x": x, "y": y, "z": z}})
    except Exception as e:
        return jsonify({"status": "ERROR", "message": f"Invalid format: {str(e)}"}), 400

@app.route('/update_obstacle', methods=['POST'])
# 시뮬레이터 환경에 추가된 Obstacle 정보를 End Point에 전달
def update_obstacle():
    data = request.get_json()
    if not data:
        return jsonify({'status': 'error', 'message': 'No data received'}), 400

    print("🪨 Obstacle Data:", data)
    return jsonify({'status': 'success', 'message': 'Obstacle data received'})

@app.route('/collision', methods=['POST']) 
# 전차와 충돌된 객체의 정보를 전달
def collision():
    data = request.get_json()
    if not data:
        return jsonify({'status': 'error', 'message': 'No collision data received'}), 400

    object_name = data.get('objectName')
    position = data.get('position', {})
    x = position.get('x')
    y = position.get('y')
    z = position.get('z')

    print(f"💥 Collision Detected - Object: {object_name}, Position: ({x}, {y}, {z})")

    return jsonify({'status': 'success', 'message': 'Collision data received'})

#Endpoint called when the episode starts
# Unity 씬이 시작될 때, 시뮬레이션 초기화 정보(탱크 위치, 진행/중지 여부 등)를 설정
@app.route('/init', methods=['GET'])
def init():
    config = {
        "startMode": "start",  # Options: "start" or "pause"
        "blStartX": 60,  #Blue Start Position
        "blStartY": 10,
        "blStartZ": 27.23,
        "rdStartX": 59, #Red Start Position
        "rdStartY": 10,
        "rdStartZ": 280,
        "trackingMode": True,
        "detactMode": False,
        "logMode": True,
        "enemyTracking": True,
        "saveSnapshot": False,
        "saveLog": True,
        "saveLidarData": False,
        "lux": 30000
    }
    print("🛠️ Initialization config sent via /init:", config)
    return jsonify(config)

@app.route('/start', methods=['GET'])
# 에피소드가 중지상태일 때 1초에 한번씩 요청하면서 시작명령을 처리
def start():
    print("🚀 /start command received")
    return jsonify({"control": ""})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
