import socket
import threading


class NetworkLoggingServer:
    def __init__(self, host, port, callback=None) -> None:
        self.host = host
        self.port = port

        self.callback = callback if callback else print

        threading.Thread(target=self.start_server, daemon=True, args=[self.host, self.port]).start()

    def start_server(self, host, port):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((host, port))
        server_socket.listen()

        print(f"Logging Server listening on {host}:{port}")

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Accepted connection from {client_address}")

            # Start a new thread to handle the client
            threading.Thread(target=self.handle_client, daemon=True, args=(client_socket,)).start()

    def handle_client(self, client_socket):
        with client_socket:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break

                log_message = data.decode("utf-8")
                self.callback(log_message)

    def stop_server(self):
        try:
            self.server_socket.close()
            print("Logging server has been stopped")
        except:
            pass

    def __del__(self):
        self.stop_server()
        super().__del__()
