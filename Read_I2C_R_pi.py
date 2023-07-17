import smbus2
import time

bus = smbus2.SMBus()
Arduino_addr = 0x04  # Задать адресс ардуино (берется из скейтча)


def read_arduino(pr):
    # Считывание оного байта из Arduino
    read_a_byte = bus.read_byte(Arduino_addr)
    if pr == 1:
        print(read_a_byte)


def read_line_a(XYI):
    # Передача команды для Arduino
    bus.write_block_data(Arduino_addr, XYI)
    contr = 0
    line = ''
    while contr == 1:
        line_byte = read_arduino(0)
        line = line + line_byte
        if "END" in line:
            contr = 1

# bus.write_block_data(Arduino_addr, data) - Передает значения в Arduino


com = input("Че те надо, блеатЪ?!1!! ")
if com == 'X':
    read_line_a('X')
if com == 'Y':
    read_line_a('Y')
if com == 'Z':
    read_line_a('Z')
if com == 'XY':
    read_line_a('XY')
if com == 'XYZ':
    read_line_a('XYZ')
