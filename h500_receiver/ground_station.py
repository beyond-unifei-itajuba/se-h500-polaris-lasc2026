import csv
import os
from datetime import datetime
from pathlib import Path
import sys

os.environ["QTWEBENGINE_REMOTE_DEBUGGING"] = "9222"

import pyqtgraph as pg
from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtSvgWidgets import QGraphicsSvgItem
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGraphicsScene,
    QGraphicsView,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from serial_worker import SerialWorker

HEADER_CSV = [
    "timestamp",
    "latitude",
    "longitude",
    "sats",
    "humidity",
    "temperatureC_dht",
    "acceleration_x",
    "acceleration_y",
    "acceleration_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "pressure",
    "altGY",
    "altMaxGY",
    "temperatureC_GY",
    "airspeed_ms",
    "expected_latitude",
    "expected_longitude",
]

# Configurações globais do pyqtgraph
pg.setConfigOptions(antialias=True)
pg.setConfigOption("background", "#121212")
pg.setConfigOption("foreground", "#A0A0A0")


class GroundStation(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("H500 - Estação de Solo HUD")
        self.resize(1300, 750)

        self.worker = None
        self.csv_file = None
        self.csv_writer = None

        self.angulo_foguete = 0.0
        self.tempo_ultimo_giro = None

        self._montar_interface()

    def _montar_interface(self):
        central = QWidget()
        central.setStyleSheet("background-color: #121212;")
        self.setCentralWidget(central)
        layout_principal = QVBoxLayout(central)
        layout_principal.setContentsMargins(15, 15, 15, 15)
        layout_principal.setSpacing(10)

        # --- Barra de conexão ---
        barra_conexao = QHBoxLayout()
        label_porta = QLabel("Porta:")
        label_porta.setStyleSheet(
            "color: #E0E0E0; font-weight: bold; font-size: 13px;"
        )

        self.input_porta = QLineEdit("COM5")
        self.input_porta.setFixedWidth(100)

        self.botao_conectar = QPushButton("Conectar")
        self.botao_conectar.clicked.connect(self._toggle_conexao)
        self.botao_conectar.setCursor(Qt.CursorShape.PointingHandCursor)

        self.label_status = QLabel("Desconectado")
        self.label_status.setStyleSheet(
            "color: #FF5555; font-weight: bold; font-size: 13px;"
        )

        barra_conexao.addWidget(label_porta)
        barra_conexao.addWidget(self.input_porta)
        barra_conexao.addWidget(self.botao_conectar)
        barra_conexao.addWidget(self.label_status)
        barra_conexao.addStretch()
        layout_principal.addLayout(barra_conexao)

        # --- Corpo: Mapa na esquerda, Cards e Gráficos na direita ---
        layout_corpo = QHBoxLayout()
        layout_principal.addLayout(layout_corpo)

        # ESQUERDA: Mapa
        self.map_view = QWebEngineView()
        caminho_mapa = Path(__file__).parent / "web" / "mapa.html"
        self.map_view.setUrl(QUrl.fromLocalFile(str(caminho_mapa)))
        layout_corpo.addWidget(self.map_view, stretch=2)

        # DIREITA: Painel Compacto
        painel_direito = QVBoxLayout()
        painel_direito.setSpacing(8)
        layout_corpo.addLayout(painel_direito, stretch=1)

        # --- Widget do foguete (tamanho reduzido para liberar espaço) ---
        self.rocket_scene = QGraphicsScene()
        caminho_svg = Path(__file__).parent / "web" / "h500.svg"
        self.rocket_item = QGraphicsSvgItem(str(caminho_svg))
        self.rocket_item.setTransformOriginPoint(
            self.rocket_item.boundingRect().center()
        )
        self.rocket_scene.addItem(self.rocket_item)

        self.rocket_view = QGraphicsView(self.rocket_scene)
        self.rocket_view.setStyleSheet(
            "background-color: #1A1A1A; border: 1px solid #333; border-radius:"
            " 4px;"
        )
        self.rocket_view.setRenderHint(self.rocket_view.renderHints())
        self.rocket_view.setMinimumHeight(160)
        self.rocket_view.setMaximumHeight(180)
        self.rocket_view.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        self.rocket_view.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        painel_direito.addWidget(self.rocket_view, stretch=0)

        from PyQt6.QtCore import QTimer

        QTimer.singleShot(
            0,
            lambda: self.rocket_view.fitInView(
                self.rocket_item, Qt.AspectRatioMode.KeepAspectRatio
            ),
        )

        # --- Gráfico de Altitude (ligeiramente menor) ---
        self.plot_altitude = pg.PlotWidget(title="Altitude Max (m)")
        self.plot_altitude.showGrid(x=True, y=True, alpha=0.2)
        self.plot_altitude.setMinimumHeight(150)
        self.plot_altitude.setMaximumHeight(200)
        self.curva_altitude = self.plot_altitude.plot(
            [], [], pen=pg.mkPen("#00E676", width=2)
        )
        painel_direito.addWidget(self.plot_altitude, stretch=0)

        # --- Grid de Cards Compactados ---
        layout_cards = QGridLayout()
        layout_cards.setSpacing(6)
        painel_direito.addLayout(layout_cards, stretch=1)

        campos_cards = [
            ("latitude", "Lat", "°"),
            ("longitude", "Lon", "°"),
            ("sats", "Sats", ""),
            ("pressure", "Pressão", "Pa"),
            ("altGY", "Alt", "m"),
            ("altMaxGY", "Alt Max", "m"),
            ("temperatureC_GY", "T(GY)", "°C"),
            ("temperatureC_dht", "T(DHT)", "°C"),
            ("airspeed_ms", "AirS", "m/s"),
            ("humidity", "Umid", "%"),
            ("expected_latitude", "Lat Pouso", "°"),
            ("expected_longitude", "Lon Pouso", "°"),
        ]

        self.cells = {}
        for i, (campo, titulo, unidade) in enumerate(campos_cards):
            card = self._criar_card_compacto(titulo, unidade)
            self.cells[campo] = card["label_valor"]
            linha, coluna = divmod(i, 2)
            layout_cards.addWidget(card["widget"], linha, coluna)

        # Buffers
        self.tempos = []
        self.altitudes = []
        self.tempo_inicio = None

    def _criar_card_compacto(self, titulo, unidade):
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        frame.setMinimumHeight(44)
        frame.setStyleSheet("""
            QFrame { 
                background-color: #1A1A1A; border-radius: 4px; border: 1px solid #333; 
            }
            QLabel#titulo { color: #888; font-size: 10px; font-weight: bold; }
            QLabel#valor { color: #FFF; font-size: 14px; font-weight: bold; }
        """)

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(1)

        label_titulo = QLabel(titulo)
        label_titulo.setObjectName("titulo")

        label_valor = QLabel(f"-- {unidade}")
        label_valor.setObjectName("valor")

        layout.addWidget(label_titulo)
        layout.addWidget(label_valor)

        return {"widget": frame, "label_valor": label_valor, "unidade": unidade}

    def _toggle_conexao(self):
        if self.worker is None:
            self._conectar()
        else:
            self._desconectar()

    def _conectar(self):
        porta = self.input_porta.text().strip()
        timestamp_execucao = datetime.now().strftime("%Y%m%d_%H%M%S")
        caminho_csv = Path(f"telemetria_{timestamp_execucao}.csv")
        self.csv_file = open(caminho_csv, "a", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(HEADER_CSV)
        self.csv_file.flush()

        self.worker = SerialWorker(porta)
        self.worker.nova_telemetria.connect(self._on_nova_telemetria)
        self.worker.erro_conexao.connect(self._on_erro_conexao)
        self.worker.start()

        self.label_status.setText(f"Conectado em {porta}")
        self.label_status.setStyleSheet(
            "color: #00E676; font-weight: bold; font-size: 13px;"
        )
        self.botao_conectar.setText("Desconectar")
        self.botao_conectar.setStyleSheet(
            "background-color: #FF5555; color: white;"
        )

    def _desconectar(self):
        if self.worker:
            self.worker.parar()
            self.worker.wait()
            self.worker = None

        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

        self.label_status.setText("Desconectado")
        self.label_status.setStyleSheet(
            "color: #FF5555; font-weight: bold; font-size: 13px;"
        )
        self.botao_conectar.setText("Conectar")
        self.botao_conectar.setStyleSheet("")

    def _on_erro_conexao(self, mensagem):
        self.label_status.setText(f"Erro: {mensagem}")
        self.label_status.setStyleSheet(
            "color: #FFB300; font-weight: bold; font-size: 13px;"
        )
        self.worker = None

    def _on_nova_telemetria(self, pacote: dict):
        timestamp = datetime.now().isoformat(timespec="milliseconds")
        linha = [timestamp] + [
            pacote.get(campo, "") for campo in HEADER_CSV[1:]
        ]
        self.csv_writer.writerow(linha)
        self.csv_file.flush()

        agora = datetime.now()
        if self.tempo_inicio is None:
            self.tempo_inicio = agora
        tempo_decorrido = (agora - self.tempo_inicio).total_seconds()

        self.tempos.append(tempo_decorrido)
        self.altitudes.append(pacote.get("altMaxGY", 0))

        self.curva_altitude.setData(self.tempos, self.altitudes)

        lat = pacote.get("latitude", 0.0)
        lon = pacote.get("longitude", 0.0)
        exp_lat = pacote.get("expected_latitude", 0.0)
        exp_lon = pacote.get("expected_longitude", 0.0)

        js_cmd = f"atualizarPosicao({lat}, {lon}, {exp_lat}, {exp_lon});"
        self.map_view.page().runJavaScript(js_cmd)

        velocidade_angular = pacote.get("gyro_z", 0.0)
        OFFSET_ORIENTACAO_SVG = 90

        if self.tempo_ultimo_giro is None:
            self.tempo_ultimo_giro = agora
        else:
            dt = (agora - self.tempo_ultimo_giro).total_seconds()
            self.tempo_ultimo_giro = agora
            self.angulo_foguete = (
                self.angulo_foguete + velocidade_angular * dt
            ) % 360
            self.rocket_item.setRotation(
                (self.angulo_foguete + OFFSET_ORIENTACAO_SVG) % 360
            )

        for campo, label in self.cells.items():
            if campo in pacote:
                valor = pacote[campo]
                if isinstance(valor, float):
                    texto = (
                        f"{valor:.4f}"
                        if "lat" in campo or "lon" in campo
                        else f"{valor:.1f}"
                    )
                else:
                    texto = str(valor)
                label.setText(texto)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    app.setStyleSheet("""
        QLineEdit {
            background-color: #1E1E1E; color: #FFF; border: 1px solid #333;
            border-radius: 4px; padding: 4px; font-size: 13px;
        }
        QLineEdit:focus { border: 1px solid #00D2FF; }
        QPushButton {
            background-color: #0078D7; color: white; border: none;
            border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 12px;
        }
        QPushButton:hover { background-color: #005A9E; }
        QPushButton:pressed { background-color: #004275; }
    """)

    janela = GroundStation()
    janela.show()
    sys.exit(app.exec())