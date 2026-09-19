from PyQt6.QtCore import QThread, pyqtSignal
import serial

class SerialWorker(QThread):
    # Sinal emitido a cada linha de telemetria válida, com os 18 campos já convertidos
    nova_telemetria = pyqtSignal(dict)
    erro_conexao = pyqtSignal(str)

    def __init__(self, porta, baudrate=115200):
        super().__init__()
        self.porta = porta
        self.baudrate = baudrate
        self._rodando = True

    def run(self):
        try:
            ser = serial.Serial(self.porta, self.baudrate, timeout=1)
        except Exception as e:
            self.erro_conexao.emit(str(e))
            return

        while self._rodando:
            try:
                linha = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not linha.startswith("DATA,"):
                continue

            valores = linha.split(",")
            if len(valores) != 19:
                continue

            dados = valores[1:]
            pacote = {
                "latitude": float(dados[0]),
                "longitude": float(dados[1]),
                "sats": int(dados[2]),
                "humidity": float(dados[3]),
                "temperatureC_dht": float(dados[4]),
                "acceleration_x": float(dados[5]),
                "acceleration_y": float(dados[6]),
                "acceleration_z": float(dados[7]),
                "gyro_x": float(dados[8]),
                "gyro_y": float(dados[9]),
                "gyro_z": float(dados[10]),
                "pressure": float(dados[11]),
                "altGY": float(dados[12]),
                "altMaxGY": float(dados[13]),
                "temperatureC_GY": float(dados[14]),
                "airspeed_ms": float(dados[15]),
                "expected_latitude": float(dados[16]),
                "expected_longitude": float(dados[17]),
            }
            self.nova_telemetria.emit(pacote)

        ser.close()

    def parar(self):
        self._rodando = False