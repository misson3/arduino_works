# Apr09, 2023, ms
# main.py
# papa-chika project

'''
soft reset: similar to hitting crtl + D
import supervisor
call supervisor.reload()

hard reset: similar to hitting reset button
import microcontroller
call microcontroller.reset()
'''

# reset
import supervisor
import time
# exit
import sys


# neokey
import board
from adafruit_neokey.neokey1x4 import NeoKey1x4

# wifi and http requests
import ssl
import wifi
import socketpool
import adafruit_requests
from mycreds import secrets

# serial
import busio


# neokey
i2c_bus = board.STEMMA_I2C()
neokey = NeoKey1x4(i2c_bus, addr=0x30)

# connect to wifi
# collect all available ssids
available_ssids = [str(network.ssid, "utf-8")
                   for network in wifi.radio.start_scanning_networks()]
print('\n--- mySSIDs ---')
for ssid in available_ssids:
    print(ssid)
print('--- ------- ---\n')
# connect to my ssid
for myssid, mypwd in secrets['ap-info']:
    if myssid in available_ssids:
        print("Connecting to %s!" % myssid)
        wifi.radio.connect(myssid, mypwd, timeout=5)
        if wifi.radio.ipv4_address:
            break
        else:
            print("Couldn't get connected to %s!" % myssid)
    else:
        print(myssid, "is not available.")

# connected?
if wifi.radio.ipv4_address:
    # indicate wifi connected status
    print("\nConnected to %s!" % myssid)
    print("Given IP address is", wifi.radio.ipv4_address)
    neokey.pixels[0] = 0x330033  # purple
    time.sleep(5)
else:
    for _ in range(100):
        print("Couldn't connect to my ssids...")
        neokey.pixels[0] = 0x333300  # yellow
        time.sleep(0.2)
        neokey.pixels[0] = 0x000000  # yellow
        time.sleep(0.2)
    print('##### no my ssid is available.  code terminated. #####')
    sys.exit()


# create requests obj
pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, ssl.create_default_context())

# UART
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=0)


#
# functions
#
def send_message(message):
    get_url = "https://api.telegram.org/bot{}".format(secrets['token'])
    get_url += "/sendMessage?chat_id={}&text={}".format(secrets['chatid'],
                                                        message)
    response = requests.get(get_url)
    print(response.text)
    response.close()


def send_message2(gmap_url):
    get_url = "https://api.telegram.org/bot{}".format(secrets['token'])
    get_url += "/sendMessage"
    json_data = {
        "chat_id": secrets['chatid'],
        "text": "📢 Papa is here! 🚗\n[Google Map URL]({})".format(gmap_url),
        "parse_mode": "markdown"
    }
    response = requests.get(get_url, json=json_data)
    print(response.text)
    response.close()


def constructGoogleMapsPinURL(line):
    dest, lat, lon, dist = line.split(',')
    url = "https://maps.google.com/maps?q={},{}".format(lat, lon)
    return url


def constructDistanceMessage(line):
    dest, lat, lon, dist = line.split(',')
    if float(dist) >= 1000:
        dist = "{:.2f}".format(float(dist)/1000) + " km"
    else:
        dist = str(int(dist)) + " m"
    msg = "📢 🚩🚗 Papa is {} away from destination: {} ⏳".format(dist, dest)
    return msg


#
# loop
#
line = "0,1.11111,2.22222,2234.56"   # dest,lat,lon,dist
# loop
while True:
    # read serial buffer
    line = uart.readline()
    if line is not None and b'\x80' not in line:
        line = line.decode('utf-8').rstrip()
        print(type(line))
        print("line from serial:", line)
        neokey.pixels[0] = 0x003300
        time.sleep(1)
        neokey.pixels[0] = 0x000000
        time.sleep(0.5)
    else:
        neokey.pixels[0] = 0x000033
        time.sleep(0.5)
        neokey.pixels[0] = 0x000000
        time.sleep(0.5)
        continue

    # line check
    if len(line.split(',')) != 4:
        neokey.pixels[0] = 0x330000
        time.sleep(0.5)
        neokey.pixels[0] = 0x000000
        time.sleep(0.5)
        continue

    # check button status
    if neokey[0]:
        # FUTURE: THIS BUTTON IS NOT VERY USEFUL.  GIVE IT ANOTHER FUNCTION.
        print("Button A: soft reset")
        neokey.pixels[0] = 0xFF0000
        time.sleep(1)
        neokey.pixels[0] = 0x0
        supervisor.reload()
    else:
        neokey.pixels[0] = 0x0

    if neokey[1]:
        print("Button B: send a fixed message")
        neokey.pixels[1] = 0xFFFF00
        f_msg = "🐔🚗　ついたぜ！　ぱ。🏁"
        send_message(f_msg)
    else:
        neokey.pixels[1] = 0x0

    if neokey[2]:
        print("Button C: send google maps pin URL")
        neokey.pixels[2] = 0x00FF00
        url = constructGoogleMapsPinURL(line)
        send_message2(url)
    else:
        neokey.pixels[2] = 0x0

    if neokey[3]:
        print("Button D: send distance with a destination name")
        neokey.pixels[3] = 0x00FFFF
        # send message
        msg = constructDistanceMessage(line)
        send_message(msg)
    else:
        neokey.pixels[3] = 0x0
