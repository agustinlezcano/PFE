import socket
import threading
import os, sys, argparse, glob, time, cv2
import numpy as np
from ultralytics import YOLO


# =======================
# CONFIG YOLO 
# =======================

POS_CAM_X_MM = 114
POS_CAM_Y_MM = 134

z_tuerca = 12
z_resorte = 13.8
z_rodamiento = 7
z_llave_fija = 3.2
z_llave_tubo = 25.5
z_tornillo = 14
z_mecha = 9.7
z_punta_destornillador = 6.3

Z_BASE = 40
Z_MARGEN_SEG = 3

parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True)
parser.add_argument('--source', required=True)
parser.add_argument('--thresh', default=0.75)
parser.add_argument('--resolution', default="1280x720")
parser.add_argument('--record', action='store_true')
parser.add_argument('--mm_px', default=0.236)

args = parser.parse_args()

model_path = args.model
img_source = args.source
min_thresh = float(args.thresh)
user_res = args.resolution
record = args.record
mm_px_factor = float(args.mm_px)


# =======================
# EVENTO GLOBAL DE CIERRE
# =======================

stop_event = threading.Event()


# =======================
# DATOS COMPARTIDOS ENTRE HILOS
# =======================

detections_lock = threading.Lock()
latest_detections = {}  # {classname: (x_mm, y_mm, z_mm)}


# =======================
# HILO 1 — SOCKET SERVER
# =======================

def socket_server():
    HOST = "127.0.0.1"
    PORT = 65432

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()

            print("Socket server esperando conexión...")
            conn, addr = s.accept()

            with conn:
                print(f"Conectado por {addr}")

                while not stop_event.is_set():
                    data = conn.recv(1024)
                    if not data:
                        break

                    cmd = data.decode().casefold()
                    print(f"Recibido: {cmd}")

                    if cmd == "bye":
                        print("Cliente cerró conexión.")
                        stop_event.set()
                        break

                    else:
                        # Buscar el objeto solicitado en las detecciones más recientes
                        with detections_lock:
                            if cmd in latest_detections:
                                x_mm, y_mm, z_mm = latest_detections[cmd]
                                raw_data = f"X{x_mm:.2f}Y{y_mm:.2f}Z{z_mm:.2f}"
                            else:
                                # Si no está detectado, responder con ceros
                                raw_data = "X0Y0Z0"
                        
                        print(f"Enviando: {raw_data}")
                        conn.sendall(raw_data.encode())

        print("Socket server finalizado.")


# =======================
# HILO 2 — YOLO DETECT
# =======================

def yolo_detect():
    print("Inicializando YOLO...")

    #Check if model file exists and is valid
    if (not os.path.exists(model_path)):
        print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
        sys.exit(0)

    # Load the model into memory and get labemap
    model = YOLO(model_path, task='detect')
    labels = model.names

    # Parse input to determine if image source is a file, folder, video, or USB camera
    img_ext_list = ['.jpg','.JPG','.jpeg','.JPEG','.png','.PNG','.bmp','.BMP']
    vid_ext_list = ['.avi','.mov','.mp4','.mkv','.wmv']

    if os.path.isdir(img_source):
        source_type = 'folder'
    elif os.path.isfile(img_source):
        _, ext = os.path.splitext(img_source)
        if ext in img_ext_list:
            source_type = 'image'
        elif ext in vid_ext_list:
            source_type = 'video'
        elif img_source.startswith('/dev/video'):
            source_type = 'usb'
            usb_idx = int(img_source.replace('/dev/video', ''))
        else:
            print(f'File extension {ext} is not supported.')
            sys.exit(0)
    elif 'usb' in img_source:
        source_type = 'usb'
        usb_idx = int(img_source[3:])
    elif img_source.startswith('/dev/video'):
        source_type = 'usb'
        usb_idx = int(img_source.replace('/dev/video', ''))
    elif 'picamera' in img_source:
        source_type = 'picamera'
        picam_idx = int(img_source[8:])
    else:
        print(f'Input {img_source} is invalid. Please try again.')
        sys.exit(0)

    # Parse user-specified display resolution
    resize = False
    if user_res:
        resize = True
        resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

    # Check if recording is valid and set up recording
    if record:
        if source_type not in ['video','usb']:
            print('Recording only works for video and camera sources. Please try again.')
            sys.exit(0)
        if not user_res:
            print('Please specify resolution to record video at.')
            sys.exit(0)
        
        # Set up recording
        record_name = 'demo1.avi'
        record_fps = 30
        recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

    # Load or initialize image source
    if source_type == 'image':
        imgs_list = [img_source]
    elif source_type == 'folder':
        imgs_list = []
        filelist = glob.glob(img_source + '/*')
        for file in filelist:
            _, file_ext = os.path.splitext(file)
            if file_ext in img_ext_list:
                imgs_list.append(file)
    elif source_type == 'video' or source_type == 'usb':

        if source_type == 'video': cap_arg = img_source
        elif source_type == 'usb': cap_arg = usb_idx
        cap = cv2.VideoCapture(cap_arg)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)  # opcional, reduce carga de CPU

        # Set camera or video resolution if specified by user
        if user_res:
            ret = cap.set(3, resW)
            ret = cap.set(4, resH)

    elif source_type == 'picamera':
        from picamera2 import Picamera2
        cap = Picamera2()
        cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
        cap.start()

    # Set bounding box colors (using the Tableu 10 color scheme)
    bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
                (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

    # Initialize control and status variables
    avg_frame_rate = 0
    frame_rate_buffer = []
    fps_avg_len = 200
    img_count = 0

    # Begin inference loop
    while True:

        t_start = time.perf_counter()

        # Load frame from image source
        if source_type == 'image' or source_type == 'folder': # If source is image or image folder, load the image using its filename
            if img_count >= len(imgs_list):
                print('All images have been processed. Exiting program.')
                sys.exit(0)
            img_filename = imgs_list[img_count]
            frame = cv2.imread(img_filename)
            img_count = img_count + 1
        
        elif source_type == 'video': # If source is a video, load next frame from video file
            ret, frame = cap.read()
            if not ret:
                print('Reached end of the video file. Exiting program.')
                break
        
        elif source_type == 'usb': # If source is a USB camera, grab frame from camera
            ret, frame = cap.read()
            if (frame is None) or (not ret):
                print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')
                break

        elif source_type == 'picamera': # If source is a Picamera, grab frames using picamera interface
            frame = cap.capture_array()
            if (frame is None):
                print('Unable to read frames from the Picamera. This indicates the camera is disconnected or not working. Exiting program.')
                break

        # Resize frame to desired display resolution
        if resize == True:
            frame = cv2.resize(frame,(resW,resH))

        # Run inference on frame
        results = model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Initialize variable for basic object counting example
        object_count = 0

        # Go through each detection and get bbox coords, confidence, and class
        for i in range(len(detections)):

            # Get bounding box coordinates
            # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
            xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
            xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
            xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

            # Get bounding box class ID and name
            classidx = int(detections[i].cls.item())
            classname = labels[classidx]

            # Get bounding box confidence
            conf = detections[i].conf.item()

            # Draw box if confidence threshold is high enough
            if conf > float(min_thresh):

                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

                label = f'{classname}: {int(conf*100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

                #Centro de la bounding box
                cx = (xmin + xmax) // 2
                cy = (ymin + ymax) // 2
                #Eje X e Y en la misma direccion que el robot
                cx_new = resH // 2 - cy 
                cy_new = resW // 2 - cx
                #Centro de la bounding box en mm (asumiendo 0.25 mm/px), despues ajustar segun calibracion
                x_mm = cx_new * float(mm_px_factor *1.32) + POS_CAM_X_MM #(float(mm_px_factor)+0.105)
                y_mm = cy_new * float(mm_px_factor) + POS_CAM_Y_MM
                
                #Asignacion de la altura Z segun el objeto
                z_mm = Z_BASE #z base y despues se le suma la altura del objeto +Z de margen de seguridad
                if classname == 'tuerca':
                    z_mm += z_tuerca
                elif classname == 'resorte':
                    z_mm += z_resorte
                elif classname == 'rodamiento':
                    z_mm += z_rodamiento
                elif classname == 'llave_fija':
                    z_mm += z_llave_fija
                elif classname == 'llave_tubo':
                    z_mm += z_llave_tubo
                elif classname == 'tornillo':
                    z_mm += z_tornillo
                elif classname == 'mecha':
                    z_mm += z_mecha
                elif classname == 'punta_destornillador':
                    z_mm += z_punta_destornillador
                
                z_mm += Z_MARGEN_SEG # Margen de seguridad para asegurar que el robot no choque con el objeto

                # Guardar detección en datos compartidos entre hilos
                with detections_lock:
                    latest_detections[classname] = (x_mm, y_mm, z_mm)

                # Draw center points and info text            
                #cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)
                cv2.drawMarker(frame, (cx, cy), (0,0,0), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=3, line_type=cv2.LINE_AA) # Use LINE_AA for anti-aliasing
                cv2.drawMarker(frame, (cx, cy), (0,255,0), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA) # Use LINE_AA for anti-aliasing
                
                cv2.drawMarker(frame, (resW // 2, resH // 2), (0,0,0), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=1, line_type=cv2.LINE_AA) # Use LINE_AA for anti-aliasing

                #cv2.putText(frame, f'({cx_new},{cy_new})', (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                cv2.putText(frame, f'({x_mm:.1f}; {y_mm:.1f}; {z_mm:.1f})mm', (cx - 90, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
                cv2.putText(frame, f'({x_mm:.1f}; {y_mm:.1f}; {z_mm:.1f})mm', (cx - 90, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                # Basic example: count the number of objects in the image
                object_count = object_count + 1

        # Calculate and draw framerate (if using video, USB, or Picamera source)
        if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
            cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
        
        # Display detection results
        cv2.putText(frame, f'Objetos detectados: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
        cv2.imshow('YOLO detection results',frame) # Display image
        if record: recorder.write(frame)

        # If inferencing on individual images, wait for user keypress before moving to next image. Otherwise, wait 5ms before moving to next frame.
        if source_type == 'image' or source_type == 'folder':
            key = cv2.waitKey()
        elif source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
            key = cv2.waitKey(5)
        
        if key == ord('q') or key == ord('Q'): # Press 'q' to quit
            stop_event.set()
            break
        elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
            cv2.waitKey()
        elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
            cv2.imwrite('capture.png',frame)
        
        # Calculate FPS for this frame
        t_stop = time.perf_counter()
        frame_rate_calc = float(1/(t_stop - t_start))

        # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
        if len(frame_rate_buffer) >= fps_avg_len:
            temp = frame_rate_buffer.pop(0)
            frame_rate_buffer.append(frame_rate_calc)
        else:
            frame_rate_buffer.append(frame_rate_calc)

        # Calculate average FPS for past frames
        avg_frame_rate = np.mean(frame_rate_buffer)


    # Clean up
    print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
    if source_type == 'video' or source_type == 'usb':
        cap.release()
    elif source_type == 'picamera':
        cap.stop()
    if record: recorder.release()
    cv2.destroyAllWindows()
    
    print("YOLO detect finalizado.")


# =======================
# MAIN — LANZA LOS HILOS
# =======================

def main():
    t_socket = threading.Thread(target=socket_server, daemon=True)
    t_yolo = threading.Thread(target=yolo_detect, daemon=True)

    t_socket.start()
    t_yolo.start()

    # Espera hasta que alguno pida cerrar
    while not stop_event.is_set():
        time.sleep(0.1)

    print("Cerrando programa...")


if __name__ == "__main__":
    main()
