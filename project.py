import time
import datetime
from bmp280 import BMP280
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt
import wiringpi
import threading

# Set desired lux and temperature
desired_lux = 20.00
desired_temperature = 19.0

# Print start message
print("Start")
pin2 = 2  # Pin voor LED
pause_time = 0.1  # Delay for LED intensity change

# Create I2C bus
bus = SMBus(0)
bmp_address = 0x76 # BMP280 address
bh1750_address = 0x23 # BH1750 address

# Setup BMP280
bmp280 = BMP280(i2c_addr= bmp_address, i2c_dev=bus)
interval = 15

# Setup BH1750
bus.write_byte(bh1750_address, 0x10)
bytes_read = bytearray(2)

# Setup wiringpi
wiringpi.wiringPiSetup()
wiringpi.softPwmCreate(pin2, 0, 100)  # Set pin as softPWM output

# Resistor pin for heating
resistor_pin = 3
wiringpi.pinMode(resistor_pin, wiringpi.OUTPUT)

# PWM parameters
MAX_DUTY_CYCLE = 100.0 # Maximum duty cycle value
MIN_DUTY_CYCLE = 0.0 # Minimum duty cycle value
proportional_gain = 1.0
alpha = 0.1  # Smoothing factor

# Initial duty cycle
previous_duty_cycle = 50.0  # Initial duty cycle value voor start

# Heating parameters
heating_threshold = 1.0  # Threshold temperature difference needed for heating

# MQTT settings
MQTT_HOST ="mqtt3.thingspeak.com"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL =60
MQTT_TOPIC = "channels/2459555/publish"
MQTT_CLIENT_ID = "JhwsLSMbCDYsFC0aASA0AyU"
MQTT_USER = "JhwsLSMbCDYsFC0aASA0AyU"
MQTT_PWD = "TZYBlFdhcVENa3/KyRVCEpnM"

# Callback functions
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("Connected OK with result code "+str(rc))
    else:
        print("Bad connection with result code "+str(rc))

# Callback function for disconnection
def on_disconnect(client, userdata, flags, rc=0):
    print("Disconnected result code "+str(rc))

# Callback function for message
def on_message(client,userdata,msg):
    print("Received a message on topic: " + msg.topic + "; message: " + msg.payload)

# Function to get lux value
def get_lux(bus, bh1750_address):
    write = i2c_msg.write(bh1750_address, [0x10]) # 1lx resolution 120ms see datasheet
    read = i2c_msg.read(bh1750_address, 2)
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    return (((bytes_read[0]&3)<<8) + bytes_read[1])/1.2 # conversion see datasheet

# Function to control LED
def controlLED(sig, cnt):
    wiringpi.softPwmWrite(sig, cnt)

# Function to update PWM signal with smoothing
def update_pwm_smooth(duty_cycle, alpha):
    global previous_duty_cycle
    # Scale duty cycle from range [0, 100] to [0, 255]
    brightness = int(duty_cycle * 2.55)
    smoothed_duty_cycle = alpha * duty_cycle + (1 - alpha) * previous_duty_cycle
    previous_duty_cycle = smoothed_duty_cycle
    controlLED(pin2, int(smoothed_duty_cycle * 2.55))
    print("Updating PWM with duty cycle:", smoothed_duty_cycle)

# Define Lux thread
def pwm_update_thread():
    while True:
        # Measure light level in thread
        bh1750_lux = get_lux(bus, bh1750_address)

        # Calculate error
        error = desired_lux - bh1750_lux

        # Proportional control
        duty_cycle = previous_duty_cycle + proportional_gain * error

        # Clamp duty cycle to valid range
        duty_cycle = max(min(duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE)

        # Update PWM signal with the calculated duty cycle using smoothing
        update_pwm_smooth(duty_cycle, alpha)

        time.sleep(0.2)  # Wait before the next reading

# Define Heating thread
def heat_resistor_thread():
    global heating # sets global variable that can be accessed outside of the thread
    heating = 0
    while True:
        current_temp = bmp280.get_temperature()
        
        if desired_temperature - current_temp > heating_threshold:
            wiringpi.digitalWrite(resistor_pin, wiringpi.HIGH)
            heating = 1
        else:
            wiringpi.digitalWrite(resistor_pin, wiringpi.LOW)
            heating = 0
        
        time.sleep(1)

# Create and start PWM update thread
pwm_thread = threading.Thread(target=pwm_update_thread)
pwm_thread.daemon = True  # Daemonize the thread so it automatically terminates when the main program ends
pwm_thread.start()

# Create and start Heating thread
heating_thread = threading.Thread(target=heat_resistor_thread)
heating_thread.daemon = True # Daemonize the thread so it automatically terminates when the main program ends
heating_thread.start()

# Set up a MQTT Client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USER, MQTT_PWD)

# Connect callback handlers to client
client.on_connect= on_connect
client.on_disconnect= on_disconnect
client.on_message= on_message

# Connect to the broker
print("Attempting to connect to %s" % MQTT_HOST)
client.connect(MQTT_HOST, MQTT_PORT)
client.loop_start() #start the loop

# Main loop
try:
    while True:
        # Measure data
        bmp280_temperature = bmp280.get_temperature()
        bmp280_pressure = bmp280.get_pressure()
        bh1750_lux = get_lux(bus, bh1750_address)
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Create box-like structure for data
        print("┌──────────────────────────────────────────────────────────────┐")
        print("│                 Data at {}                  │".format(current_time))
        print("├──────────────────────────────────────────────────────────────┤")
        print("│    Temperature: {:4.1f} °C, Pressure: {:4.1f} hPa, Lux: {:4.1f}     │".format(bmp280_temperature, bmp280_pressure, bh1750_lux))
        print("└──────────────────────────────────────────────────────────────┘")
        # Create the JSON data structure
        MQTT_DATA = "field1=" + str(bmp280_temperature) + "&field2=" + str(bh1750_lux) + "&field3=" + str(desired_lux) + "&field4=" + str(desired_temperature) + "&field5=" + str(heating) + "&status=MQTTPUBLISH"
        print(MQTT_DATA)

        # Publish the data to the broker
        try:
            client.publish(topic=MQTT_TOPIC, payload=MQTT_DATA, qos=0, retain=False, properties=None)
            time.sleep(interval)
        except OSError:
            client.reconnect()

# Stop the PWM signal and print done message
except KeyboardInterrupt:
    wiringpi.softPwmWrite(pin2, 0)  # Stop the PWM output
    print("\nDone")
    