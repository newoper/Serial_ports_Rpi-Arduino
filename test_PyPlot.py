# https://devpractice.ru/matplotlib-lesson-1-quick-start-guide/

import matplotlib.pyplot as plt
import numpy as np

# Линейная зависимость
x = np.linspace(0, 10, 50)
y1 = x
# Квадратичная зависимость
y2 = [i**2 for i in x]\

# Построение графика
# plt.title("Линейная зависимость y = x")  # заголовок
# plt.xlabel("x")  # ось абсцисс
# plt.ylabel("y")  # ось ординат
# plt.grid()      # включение отображение сетки
# plt.plot(x, y1, "r--", x, y2, "b")  # построение графика
# plt.show()

# Построение графиков
# figure() – функция для задания глобальных параметров отображения
# графиков. В нее, в качестве аргумента, мы передаем кортеж,
# определяющий размер общего поля.
plt.figure(figsize=(9, 9))

# subplot() – функция для задания местоположения поля с графиком.
# Существует несколько способов задания областей для вывода через
# функцию subplot() мы воспользовались следующим:
# первый аргумент – количество строк, второй – столбцов в формируемом поле,
# третий – индекс (номер поля, считаем сверху вниз, слева направо).

plt.subplot(2, 1, 1)
plt.plot(x, y1)
plt.title("Це график")
plt.ylabel("y1", fontsize=14)  # fontsize для задания размера шрифта.
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(x, y2)
plt.xlabel("x", fontsize=14)
plt.ylabel("y2", fontsize=14)
plt.grid(True)
plt.minorticks_on()  # Добавление делений на шкале
plt.tick_params(which='major', length=10, width=2)
plt.tick_params(which='minor', length=5, width=1)
plt.show()

# Построение диаграмм 
fruits = ["apple", "peach", "orange", "bannana", "melon"]
counts = [34, 25, 43, 31, 17]
plt.bar(fruits, counts)
plt.title("Fruits!")
plt.xlabel("Fruit")
plt.ylabel("Count")
plt.show()
print('~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~')
