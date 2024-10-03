# from driver import RealSenseCamera

# if __name__ == "__main__":
#     camera = RealSenseCamera()
#     camera.save_frame_data('frame_data')
#     camera.stop()

import argparse
from driver import RealSenseCamera

if __name__ == "__main__":
    # Настройка парсера аргументов
    parser = argparse.ArgumentParser(description='Сохранение данных с камеры RealSense.')
    parser.add_argument('filename', type=str, help='Имя файла для сохранения данных кадров')
    # Парсинг аргументов
    args = parser.parse_args()
    try:
        # Инициализация камеры
        camera = RealSenseCamera()
        # Сохранение данных кадров с указанным именем файла
        data = camera.get_frame_data()
        camera.save_frame_data(args.filename, data)
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    finally:
        # Остановка камеры
        camera.stop()
