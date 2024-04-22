"""
See documentation at https://nRF24.github.io/pyRF24
"""
import json
import requests
import threading
import time
import struct
from pyrf24 import RF24, RF24Network, RF24Mesh, RF24_DRIVER

print(file)  # print example name
def send_data(my_arr):

    endpoint = f"http://45.150.64.196:5000/master/push?token={token}"
    while True:
        time.sleep(10)  

        if len(my_arr) == 0:
            continue

        cpy = my_arr.copy()
        my_arr.clear()
        try:
            js = json.dumps(cpy)
            print(js)
            headers = {'Content-Type': 'application/json', 'Accept':'application/json'}
            response = requests.post(endpoint, data=js, headers=headers)
            if response.status_code == 200:
                print("Data sent successfully")
            else:
                print(f"Failed to send data: {response.status_code}")
        except: pass

CSN_PIN = 0  # aka CE0 on SPI bus 0: /dev/spidev0.0
if RF24_DRIVER == "MRAA":
    CE_PIN = 15  # for GPIO22
elif RF24_DRIVER == "wiringPi":
    CE_PIN = 3  # for GPIO22
else:
    CE_PIN = 22
radio = RF24(CE_PIN, CSN_PIN)
network = RF24Network(radio)
mesh = RF24Mesh(radio, network)
mesh.node_id = 0
if not mesh.begin():
    # if mesh.begin() returns false for a master node,
    # then radio.begin() returned false.
    raise OSError("Radio hardware not responding.")
radio.print_pretty_details()
my_arr = []
while True:
    token = input("Please enter your token: ")
    validation_endpoint = f"http://45.150.64.196:5000/master/validate_token?token={token}"

    response = requests.get(validation_endpoint)
    
    if response.status_code == 200:
        print("Token is valid")
        break
    else:
        print("Invalid token. Please try again.")

thread = threading.Thread(target=lambda: send_data(my_arr))
thread.start()

try:
    while True:
        mesh.update()
        mesh.dhcp()

        while network.available():
            header, payload = network.read()
            print(f"Received message {header.to_string()}")
            print(len(payload))
            format = 'iiffif10s'

            sensor_data = struct.unpack(format, payload)

            # Преобразуем строку, убирая лишние байты (например, null-байты)
            hardware_name = sensor_data[6].decode().strip('\x00')
            guid = sensor_data[0]
            dht_status = sensor_data[1]
            humidity = sensor_data[2]
            temperature = sensor_data[3]
            gas_status = sensor_data[4]
            gas_value = sensor_data[5]
            print("GUID:", guid)
            print("Hardware Name:", hardware_name)
            print("DHT Status:", sensor_data[1])
            print("Humidity:", sensor_data[2])
            print("Temperature:", sensor_data[3])
            print("Gas Status:", sensor_data[4])
            print("Gas Value:", sensor_data[5])
            sensor_data = {
                "guid": str(guid),
                "hardware_name": hardware_name,
                "values": [
                    {
                        "sensor_name": "dht",
                        "status": dht_status,
                        "sensor_data": [
                            {"sensor_name": "hum", "sensor_value": humidity},
                            {"sensor_name": "temp", "sensor_value": temperature},
                        ],
                    },
                    {
                        "sensor_name": "mq2",
                        "status": gas_status,
                        "sensor_data": [{"sensor_name": "mq2", "sensor_value": gas_value}],
                    },
                ],
            }

            # Добавляем в массив, используя структуру данных
            my_arr.append(sensor_data)
            #print(payload.decode("utf-8sizeofs"))
except KeyboardInterrupt:
    print("powering down radio and exiting.")
    radio.power = False