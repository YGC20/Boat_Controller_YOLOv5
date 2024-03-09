import subprocess
import json

command = "python3 detect.py --source 0 --weights best.pt --conf 0.25"
result = subprocess.run(command,shell=True)

# 실행 결과 확인
if result.returncode == 0:
    # 실행이 성공했을 경우, JSON 형식으로 받은 객체 정보를 파싱하여 처리
    objects_detected = json.loads(result.stdout)
    # 받은 객체 정보 출력
    for obj in objects_detected:
        print(obj)
else:
    # 실행이 실패했을 경우, 에러 메시지 출력
    print("Error:", result.stderr)
