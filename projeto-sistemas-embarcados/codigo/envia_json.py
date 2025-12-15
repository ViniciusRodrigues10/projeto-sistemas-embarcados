import time
import math
import random
import serial
import json
import influxdb_client
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS

TOKEN = ""
HOST = "http://localhost:8086/"
DATABASE = "MesaLabirinto"
ORG = "default"

# 2. Configurações da Porta Serial (USB)
# No Windows: "COM3", "COM4", etc. (Veja no Gerenciador de Dispositivos)
# No Linux/Mac: "/dev/ttyUSB0" ou "/dev/ttyACM0"
SERIAL_PORT = "COM6"  
BAUD_RATE = 115200    # Tem que ser IGUAL ao configurado no ESP32

# ================= CONEXÕES =================

print(f"🔌 Conectando ao InfluxDB em {HOST}...")
try:
    client = influxdb_client.InfluxDBClient(token=TOKEN, url=HOST, org=ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    print("✅ InfluxDB Conectado!")
except Exception as e:
    print(f"❌ Erro InfluxDB: {e}")
    exit()

print(f"🔌 Abrindo porta Serial {SERIAL_PORT}...")
try:
    # Abre a conexão com o ESP32
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Espera o Arduino/ESP reiniciar após abrir a porta
    print(f"✅ Serial Aberta! Aguardando dados JSON do ESP32...")
except Exception as e:
    print(f"❌ Erro ao abrir Serial: {e}")
    print("Dica: Verifique se a porta está correta e se o Monitor Serial do Arduino IDE está fechado.")
    exit()

# ================= LOOP PRINCIPAL =================

print("\n--- ESCUTANDO DADOS REAIS (Ctrl+C para parar) ---\n")

while True:
    try:
        # 1. Verifica se tem dados chegando na USB
        if ser.in_waiting > 0:
            
            # 2. Lê a linha bruta (bytes) e transforma em texto (string)
            linha_bytes = ser.readline()
            linha_texto = linha_bytes.decode('utf-8', errors='ignore').strip()
            
            # (Opcional) Mostra o que chegou para debug
            # print(f"Raw: {linha_texto}")

            # 3. Tenta processar APENAS se parecer um JSON (começa com { e termina com })
            if linha_texto.startswith('{') and linha_texto.endswith('}'):
                try:
                    # Converte texto para Dicionário Python
                    dados_json = json.loads(linha_texto)

                    # 4. Prepara o Ponto para o InfluxDB
                    ponto = Point("mesa_status") \
                        .tag("origem", "esp32_hardware_real") \
                        .field("joy_x", int(dados_json["joy_x"])) \
                        .field("joy_y", int(dados_json["joy_y"])) \
                        .field("servo_x", int(dados_json["servo_x"])) \
                        .field("servo_y", int(dados_json["servo_y"])) \
                        .field("pitch", float(dados_json["pitch"])) \
                        .field("roll", float(dados_json["roll"])) \
                        .field("gyro_x", int(dados_json["gyro_x"])) \
                        .field("gyro_y", int(dados_json["gyro_y"])) \
                        .field("gyro_z", int(dados_json["gyro_z"]))

                    # 5. Grava no Banco
                    write_api.write(bucket=DATABASE, org=ORG, record=ponto)
                    
                    # Feedback visual no terminal
                    print(f"📡 Gravado: Pitch: {dados_json['pitch']} | ServoX: {dados_json['servo_x']}")

                except json.JSONDecodeError:
                    # Acontece se o JSON vier quebrado ou incompleto pela metade
                    print(f"⚠️ Erro de JSON: {linha_texto}")
            else:
                # Ignora mensagens de log do ESP32 (ex: "Starting...", "WiFi Connected")
                pass

    except KeyboardInterrupt:
        print("\n🛑 Ponte Serial Encerrada.")
        ser.close()
        break
    except Exception as e:
        print(f"❌ Erro inesperado: {e}")
        # Tenta reconectar ou segue