# 두 번째 터미널에서 실행되는 코드 (detect_table_movement.py)
import sys

def detect_table_movement():
    # 어떠한 방법으로 'moving to a table'을 감지하는 코드 작성
    # 감지되면 다음과 같이 실행
    print("moving to a table detected")
    sys.stdout.flush()  # 버퍼를 강제로 비워서 출력을 즉시 터미널에 표시

if __name__ == "__main__":
    detect_table_movement()
