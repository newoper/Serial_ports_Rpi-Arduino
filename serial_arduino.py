import sys
import glob
import serial
from serial import Serial


def read_arduino(port_x):
    """Последовательно выводит значения с заданного порта
    """
    with Serial(port_x, 9600) as ser:
        i = 0
        while i < 50:
            i += 1
            line = ser.readline()
            print(line)


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


port = serial_ports()
# port.reverse()
print(port)


for port_arduino in port:
    """Определяет порт Arduino путем поиска ключевого слова в начале строки.
    Если порт не отвечает более 2 секунд, то пропускает его.
    Если порт найден, то выводит значения с заданного порта.
    Если не находит Arduino, то выводит "Не найдено подключение Arduino"
    """
    print('Идет работа с портом:', port_arduino)
    with Serial(port_arduino, 9600, timeout=2) as ser:
        # print(ser)
        line = ''
        line = str(ser.readline())
        if len(line) < 1:
            print(line)
        ser.close()
        if "Arduino" in line:
            port_a = port_arduino
            print('Определился:', port_a)
            # print(type(port_a))
            read_arduino(port_a)
        else:
            print('Не найдено подключение Arduino ' + str(port_arduino))

print('~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~')
