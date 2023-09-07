import tkinter as tk
import threading
from tkinter import filedialog, PhotoImage, messagebox
from pymavlink import mavutil
import serial.tools.list_ports
import time

# Глобальные переменные
ardupilot_port = None
parameter_loading = False
show_status = "TEST"
file_path = None
error_message = None


# =========MAVLINK================
def find_ardupilot_port():
    global ardupilot_port

    # Поиск доступных COM-портов
    available_ports = list(serial.tools.list_ports.comports())

    # Поиск порта с нужным Caption
    for port in available_ports:
        print(port.description, port.device)
        if "MatekH743-bdshot" in port.description:
            ardupilot_port = port.device
            print("Найден дрона на порту:", ardupilot_port)
            return
    # Если не найден порт с нужным Caption
    error = "ArduPilot не найден. Убедитесь, что он подключен и правильно распознается"
    messagebox.showerror("ArduPilot Error", error)
    raise ValueError(error)


def mavlink_receive_thread():
    master = mavutil.mavlink_connection(ardupilot_port, baud=57600)

    while True:
        try:
            msg = master.recv_msg()
            if msg is not None:
                # Process the received MAVLink message
                text_widget.insert(tk.END, msg)
                text_widget.see(tk.END) 
        except KeyboardInterrupt:
            print("Exiting...")
            break


def load_parameters():
    global ardupilot_port
    global error_message

    try:
        # Поиск порта ArduPilot, если он еще не найден
        if not ardupilot_port:
            find_ardupilot_port()

        # Устанавливаем соединение с ArduPilot
        master = mavutil.mavlink_connection(ardupilot_port)

        mavlink_thread = threading.Thread(target=mavlink_receive_thread)
        mavlink_thread.daemon = (
            True  # Daemonize the thread to exit when the main program exits
        )
        mavlink_thread.start()

        # Открываем файл с параметрами
        with open(file_path, "r") as file:
            parameters = file.read()

        # Разделяем параметры построчно
        parameter_list = parameters.splitlines()

        # Загружаем параметры на ArduPilot
        for param in parameter_list:
            name, value = param.split(",")
            master.param_set_send(name, float(value))

        print("Параметры успешно загружены на ArduPilot")

    except Exception as e:
        print("Произошла ошибка:", e)


def spinMotor(master, motor_to_test=1):
    throttle_percentage = 10  # 50% throttle for the test
    test_mode = 0  # Motor test mode
    test_timeout = 1  # Timeout between tests that are run in sequence.
    motor_order = 0  # Default motor order
    motor_count = 1

    # Send the MAV_CMD_DO_MOTOR_TEST command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,  # Confirmation
        motor_to_test,
        test_mode,
        throttle_percentage,
        test_timeout,
        motor_count,
        motor_order,
        0,  # Reserved
    )

def handleMotorSpin(motor_to_test):
    try:
        if not ardupilot_port:
            find_ardupilot_port()

        master = mavutil.mavlink_connection(ardupilot_port)
        master.wait_heartbeat()

        spinMotor(master, motor_to_test)
    except Exception as e:
        print("Произошла ошибка при прокрутке моторов:", e)


# ================GUI================


# Создание графического интерфейса
def open_file_dialog():
    global file_path
    file_path = filedialog.askopenfilename(filetypes=[("All Files", "*.param")])
    if file_path:
        file_path_label.config(text="Выбранный файл: " + file_path)
    else:
        file_path_label.config(text="Файл не выбран")


# Создание графического интерфейса
root = tk.Tk()
root.title("Выбор файла")
root.geometry("800x700")

open_button = tk.Button(root, text="Выбрать файл", command=open_file_dialog)
open_button.pack(padx=20, pady=20)

file_path_label = tk.Label(root, text="Файл не выбран", fg="blue")
file_path_label.pack()

upload_button = tk.Button(root, text="Загрузить параметры", command=load_parameters())
upload_button.pack(padx=20, pady=20)

img = PhotoImage(file="./motors.png")
# resized_image = img.subsample(1, 1)
label = tk.Label(root, image=img)
label.pack(padx=20, pady=10, side=tk.TOP)

button_frame = tk.Frame(root)
button_frame.pack()
button_idx_to_id = [3, 1, 2, 4]

for motor in range(1, 5):
    row = (motor - 1) // 2
    col = (motor - 1) % 2
    id = button_idx_to_id[motor - 1]
    button_map = {1: 1, 2: 3, 3: 4, 4: 2}

    motor_button = tk.Button(
        button_frame,
        text=f"Мотор {id}",
        command=lambda id=id: handleMotorSpin(button_map[id]),
    )
    motor_button.grid(row=row, column=col, padx=1, pady=2)

text_widget = tk.Text(root, wrap=tk.WORD, height=10, width=100)
text_widget.pack(padx=20, pady=10)

root.mainloop()
