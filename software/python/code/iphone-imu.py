import requests
import time

url = "http://192.168.1.99:8000/data"  # replace with your iPhone's IP

while True:
    try:
        response = requests.get(url, timeout=1)
        data = response.text.strip().split('\n')[-1]  # get latest row
        print("Latest IMU reading:", data)
    except Exception as e:
        print("Error:", e)
    time.sleep(0.1)
