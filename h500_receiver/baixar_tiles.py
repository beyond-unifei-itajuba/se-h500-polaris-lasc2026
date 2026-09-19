"""
Baixa tiles do OpenStreetMap para uso offline no Leaflet.

USO:
    1. Ajuste as variáveis LAT_MIN, LAT_MAX, LON_MIN, LON_MAX e ZOOM_MIN/ZOOM_MAX abaixo.
    2. Rode: python baixar_tiles.py
    3. As tiles vão pra pasta ./tiles/{z}/{x}/{y}.png
    4. Copie a pasta "tiles" pra dentro da sua pasta "web/", do lado do mapa.html.

IMPORTANTE - Uso responsável do servidor de tiles do OSM:
    O servidor público tile.openstreetmap.org é mantido por voluntários e tem uma
    política de uso (https://operations.osmfoundation.org/policies/tiles/) que pede:
      - Não baixar em massa (bulk downloading) sem necessidade real
      - Respeitar um limite de ~2 requisições/segundo
      - Identificar seu app com um User-Agent claro
      - Não usar para produção/alto volume - pra isso, use um provedor pago (Mapbox,
        MapTiler, Stadia Maps, etc.) ou rode seu próprio servidor de tiles.
    Este script já limita a taxa de requisições e usa um User-Agent identificável.
    Para uma área pequena (raio de alguns km ao redor do local de lançamento) e
    poucos níveis de zoom, isso é uso pessoal/hobby normal e aceitável.
"""

import math
import os
import time
import urllib.request
import urllib.error

# ======================= CONFIGURAÇÃO =======================
# Lista de áreas a baixar - adicione quantas quiser.
# Em vez de calcular manualmente lat/lon min/max (fácil de errar e acabar
# com uma área minúscula sem querer), você informa:
#   - "nome": só identificação
#   - "lat", "lon": o PONTO CENTRAL da área (ex: local de lançamento/pouso)
#   - "raio_km": quantos km pra cada lado a partir do centro você quer cobrir
#                (ex: 5 = uma área de aprox. 10km x 10km ao redor do ponto)
AREAS = [
    {
        "nome": "Iacanga",
        "lat": -21.897231,   # <-- ponto central (ajuste se necessário)
        "lon": -49.022262,   # <-- ponto central (ajuste se necessário)
        "raio_km": 5,
    },
    {
        "nome": "UNIFEI",
        "lat": -22.413796,   # <-- ponto central perto de Itajubá/Pinheirinho
        "lon": -45.450268,
        "raio_km": 2,        # <-- ajuste o raio conforme a precisão que você precisa
    },
    {
        "nome": "Iacanga - SCLR",
        "lat": -21.940389,
        "lon": -48.942917,
        "raio_km": 5,
    },
    {   "nome": "Bauru - Acampamento",
        "lat": -22.2371579,
        "lon": -49.0508638,
        "raio_km": 2,
    },
]

# Níveis de zoom a baixar (quanto maior, mais detalhe e mais tiles) - vale pra todas as áreas
ZOOM_MIN = 10
ZOOM_MAX = 16

# Pasta de saída - já aponta direto pra dentro de web/, do lado do mapa.html,
# assim você nunca precisa mover a pasta manualmente (e o script sempre
# reconhece o que já foi baixado antes, mesmo rodando de novo).
PASTA_SAIDA = os.path.join("web", "tiles")

# Servidor de tiles (troque se quiser usar outro provedor)
URL_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

# Identifique seu projeto aqui (boa prática / exigido por muitos provedores)
USER_AGENT = "H500-EstacaoDeSolo/1.0 (uso pessoal - contato: seu-email@exemplo.com)"

# Pausa entre requisições, em segundos (respeita o limite do OSM de ~2 req/s)
# Se você tomar bloqueios (erro 429), aumente esse valor (ex: 1.0 ou 1.5)
PAUSA_SEGUNDOS = 1.0

# Arquivo onde ficam registrados os tiles que falharam, pra re-tentar depois
ARQUIVO_FALHAS = "tiles_faltando.txt"
# ==============================================================


def deg2num(lat_deg, lon_deg, zoom):
    """Converte lat/lon para índices de tile (x, y) no zoom dado."""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile


def ponto_e_raio_para_bbox(lat, lon, raio_km):
    """Converte um ponto central + raio (em km) numa bbox (lat/lon min/max)."""
    km_por_grau_lat = 111.32  # aprox., constante em qualquer latitude
    km_por_grau_lon = 111.32 * math.cos(math.radians(lat))  # varia com a latitude

    delta_lat = raio_km / km_por_grau_lat
    delta_lon = raio_km / km_por_grau_lon

    return {
        "lat_min": lat - delta_lat,
        "lat_max": lat + delta_lat,
        "lon_min": lon - delta_lon,
        "lon_max": lon + delta_lon,
    }


def baixar_tile(z, x, y, pasta_saida):
    caminho_pasta = os.path.join(pasta_saida, str(z), str(x))
    os.makedirs(caminho_pasta, exist_ok=True)
    caminho_arquivo = os.path.join(caminho_pasta, f"{y}.png")

    if os.path.exists(caminho_arquivo):
        return "existente", None

    url = URL_TEMPLATE.format(z=z, x=x, y=y)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})

    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            dados = resp.read()
        with open(caminho_arquivo, "wb") as f:
            f.write(dados)
        return "baixado", None
    except urllib.error.HTTPError as e:
        # 429 = "Too Many Requests" -> o servidor te bloqueou temporariamente
        # 403 = acesso negado -> também costuma ser bloqueio por uso indevido
        motivo = f"HTTP {e.code}"
        if e.code == 429:
            motivo += " (BLOQUEIO por excesso de requisições - aumente PAUSA_SEGUNDOS)"
        print(f"\n  [ERRO] z={z} x={x} y={y}: {motivo}")
        return "erro", (z, x, y)
    except Exception as e:
        print(f"\n  [ERRO] z={z} x={x} y={y}: {e}")
        return "erro", (z, x, y)


def formatar_tempo(segundos):
    minutos, seg = divmod(int(segundos), 60)
    horas, minutos = divmod(minutos, 60)
    if horas:
        return f"{horas}h{minutos:02d}m{seg:02d}s"
    return f"{minutos}m{seg:02d}s"


def main():
    # ---- Primeiro passo: calcula o plano de download de TODAS as áreas e zooms ----
    plano = []  # lista de (nome_area, zoom, x_min, x_max, y_min, y_max, qtd)
    total_geral = 0
    for area in AREAS:
        bbox = ponto_e_raio_para_bbox(area["lat"], area["lon"], area["raio_km"])
        for zoom in range(ZOOM_MIN, ZOOM_MAX + 1):
            x_min, y_max = deg2num(bbox["lat_min"], bbox["lon_min"], zoom)
            x_max, y_min = deg2num(bbox["lat_max"], bbox["lon_max"], zoom)
            x_min, x_max = sorted((x_min, x_max))
            y_min, y_max = sorted((y_min, y_max))
            qtd_tiles = (x_max - x_min + 1) * (y_max - y_min + 1)
            plano.append((area["nome"], zoom, x_min, x_max, y_min, y_max, qtd_tiles))
            total_geral += qtd_tiles

    print(f"Áreas configuradas: {', '.join(a['nome'] for a in AREAS)}")
    print(f"Total de tiles a processar (todas as áreas): {total_geral}\n")

    total_baixado = 0
    total_existente = 0
    total_erro = 0
    processados = 0
    inicio = time.time()
    falhas = []

    for nome_area, zoom, x_min, x_max, y_min, y_max, qtd_tiles in plano:
        print(f"\n[{nome_area}] Zoom {zoom}: {qtd_tiles} tiles (x: {x_min}-{x_max}, y: {y_min}-{y_max})")

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                resultado, info_falha = baixar_tile(zoom, x, y, PASTA_SAIDA)
                if resultado == "baixado":
                    total_baixado += 1
                    time.sleep(PAUSA_SEGUNDOS)  # só espera quando baixa de fato
                elif resultado == "existente":
                    total_existente += 1
                else:
                    total_erro += 1
                    falhas.append(info_falha)

                processados += 1

                # Barra de progresso simples, atualizada na mesma linha
                pct = processados / total_geral * 100
                decorrido = time.time() - inicio
                if total_baixado > 0:
                    tempo_por_tile = decorrido / processados
                    restante = tempo_por_tile * (total_geral - processados)
                    eta_txt = formatar_tempo(restante)
                else:
                    eta_txt = "?"

                largura_barra = 30
                preenchido = int(largura_barra * processados / total_geral)
                barra = "#" * preenchido + "-" * (largura_barra - preenchido)

                print(
                    f"\r[{barra}] {processados}/{total_geral} ({pct:5.1f}%) "
                    f"| baixadas: {total_baixado} existentes: {total_existente} erros: {total_erro} "
                    f"| ETA: {eta_txt}   ",
                    end="", flush=True
                )

        print()  # pula linha ao terminar cada zoom/área

    print("\n===== RESUMO =====")
    print(f"Tiles baixadas agora: {total_baixado}")
    print(f"Tiles já existentes:  {total_existente}")
    print(f"Erros:                {total_erro}")
    print(f"Tempo total:          {formatar_tempo(time.time() - inicio)}")
    print(f"\nPasta final: ./{PASTA_SAIDA}/  -> copie para web/tiles/")

    if falhas:
        with open(ARQUIVO_FALHAS, "w") as f:
            for z, x, y in falhas:
                f.write(f"{z},{x},{y}\n")
        print(f"\n⚠ {len(falhas)} tiles falharam e foram registradas em '{ARQUIVO_FALHAS}'.")
        print("  Rode o script novamente (ele pula o que já existe) para tentar de novo.")
        print("  Se continuar falhando, aumente PAUSA_SEGUNDOS (ex: 1.5 ou 2.0) e tente mais tarde.")


if __name__ == "__main__":
    main()