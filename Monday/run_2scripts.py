import subprocess

p1 = subprocess.Popen(['python', 'ESP32cam.py'])
p2 = subprocess.Popen(['python', 'UI_Bluetooth.py'])

p1.wait()
p2.wait()
