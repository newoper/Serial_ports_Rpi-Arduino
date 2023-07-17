import sys
import glob
import serial
from serial import Serial


def serial_ports():
    """Перечисляет имена последовательных портов.

         : вызывает EnvironmentError:
             На неподдерживаемых или неизвестных платформах
         : возвращает:
             Список последовательных портов, доступных в системе.
     st of the serial ports available on the system
    """

    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def read_arduino():
    with Serial('COM4', 9600) as ser:
        i = 0
        while i < 50:
            i += 1
            line = ser.readlines()
            print(line)
    ser.close()


port = serial_ports()
print(port)
print(len(port))
"""
Определение нужного порта автоматом?

"""

for port_arduino in port:
    try:
        with Serial('port_arduino', 9600) as ser:
            line = ser.readline()
            if line == "b'Arduino\r\n'":
                port_a = port_arduino
            else:
                print('Не найдено подключение Arduino '+ str(port_arduino) + 'else')
        ser.close()
    except:
        print('Не найдено подключение Arduino ' + str(port_arduino))


read_arduino()
