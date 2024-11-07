from pymodbus.client import ModbusTcpClient
import time
from gpiozero import LED

RED_LED = LED(14)
GREEN_LED = LED(15)
BLUE_LED = LED(18)

def set_sig(num1, num2):
    if client.connect():
        client.write_register(num1, num2, 255)
    else:
        print("Unable to connect to the Modbus server.")

def get_sig(num1):
    if client.connect():
        read1 = client.read_holding_registers(num1, 1, 255)
        print(read1.registers[0])
        return read1.registers[0]
    else:
        print("Unable to connect to the Modbus server.")
        return None

def reset_sig():
    if client.connect():
        client.write_register(0, 0, 255)
        client.write_register(1, 0, 255)
        client.write_register(2, 0, 255)
        client.write_register(3, 0, 255)
        
        client.write_register(4, 208, 255)
        client.write_register(5, 0, 255)
        client.write_register(6, 268, 255)
    else:
        print("Unable to connect to the Modbus server.")

client = ModbusTcpClient('192.168.26.32', port=502)

reset_sig()

try:
    while True:
        R = get_sig(100)
        G = get_sig(101)
        B = get_sig(102)

        time.sleep(1)

        if R:
            RED_LED.on()
        else:
            RED_LED.off()

        if G:
            GREEN_LED.on()
        else:
            GREEN_LED.off()

        if B:
            BLUE_LED.on()
        else:
            BLUE_LED.off()

except KeyboardInterrupt:
    print("end")

finally:
    RED_LED.off()
    GREEN_LED.off()
    BLUE_LED.off()
    client.close()