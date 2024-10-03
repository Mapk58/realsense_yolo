from driver import RealSenseCamera
import time

if __name__ == "__main__":
    camera = RealSenseCamera()

    while True:
        try:
            detail_number = int(input("Введите номер детали (число):  "))
            x = int(input("Введите координату по X (по горизонтали):  "))
            y = int(input("Введите координату по Y (по вертикали):  "))
            seconds = int(time.time()) % 10000
            print("Съёмка...")
            data = camera.get_frame_data()
            camera.save_frame_data(f"data/{detail_number}__{x}-{y}__{seconds}", data)
            print("Успешно! Теперь переместите деталь или установите новую.")
        except:
            print("Что-то пошло не так. Попробуем еще раз.")