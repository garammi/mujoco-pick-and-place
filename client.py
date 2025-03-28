import socket

# 서버에 명령 전달
def send_command(command, host="192.168.35.23", port=8888):  # 수정된 IP와 포트
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        client_socket.send(command.encode())  # 명령 전송
        print(f"Command '{command}' sent to server.")
        client_socket.close()
    except Exception as e:
        print(f"Failed to send command: {e}")

# 사용 예시
if __name__ == "__main__":
    send_command("pick")  # pick 명령 전달
