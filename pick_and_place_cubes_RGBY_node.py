import sys
import os
import time
import cv2
import numpy as np
from ctypes import *
import threading
from flask import Flask, Response
from flask_cors import CORS

# ================================================================
# 1. CONFIGURACIÓN DE RUTAS Y CARGA DE DLL (ESTRICTO)
# ================================================================
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Necesario para Python 3.8+ en Windows para encontrar las DLLs
if sys.platform == 'win32':
    try:
        os.add_dll_directory(script_dir)
    except:
        pass

# Importamos Dobot ANTES que la cámara para evitar conflictos
import DobotDllType as dType

# Importamos Cámara Hikrobot
try:
    sys.path.append(os.path.join(script_dir, "MvImport"))
    from MvCameraControl_class import *
except ImportError as e:
    print(f"[ERROR] No se pudo cargar la librería de la cámara Hikrobot: {e}")
    sys.exit()

# ================================================================
# 2. CONFIGURACIÓN FLASK PARA VIDEO EN DASHBOARD
# ================================================================
app = Flask(__name__)
CORS(app)
global_frame = None

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if global_frame is not None:
                ret, buffer = cv2.imencode('.jpg', global_frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.04) # ~25 FPS
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    # Flask corre en un hilo separado
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

# ================================================================
# 3. CONSTANTES Y LÓGICA DE COLOR
# ================================================================
COLOR_RANGES = {
    'red': [(np.array([0, 30, 30]), np.array([10, 255, 255])), (np.array([160, 30, 30]), np.array([180, 255, 255]))],
    'blue': [(np.array([100, 30, 30]), np.array([130, 255, 255]))],
    'green': [(np.array([35, 30, 30]), np.array([85, 255, 255]))],
    'yellow': [(np.array([20, 30, 30]), np.array([35, 255, 255]))],
}

CALIBRATION_RAIL_POS = 200
SAFE_Z = 70.0
CUBE_Z = 7.0

def apply_color_mask(hsv_frame, color_name):
    mask = None
    ranges = COLOR_RANGES.get(color_name, [])
    for (lower, upper) in ranges:
        curr_mask = cv2.inRange(hsv_frame, lower, upper)
        mask = curr_mask if mask is None else cv2.add(mask, curr_mask)
    if mask is not None:
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# ================================================================
# 4. FUNCIÓN SETUP DOBOT
# ================================================================
def setup_dobot():
    try:
        api = dType.load()
        print("[INFO] Conectando a Dobot en COM3...")
        state = dType.ConnectDobot(api, "COM3", 115200)[0]
        
        if state == dType.DobotConnect.DobotConnect_NoError:
            print("[OK] Dobot Conectado con exito.")
            dType.SetQueuedCmdClear(api)
            
            # Homing Inicial
            print("[INFO] Realizando Homing...")
            dType.SetHOMEParams(api, 200, 0, 0, 0, isQueued=1)
            dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
            dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
            lastIndex = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
            
            dType.SetQueuedCmdStartExec(api)
            while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                dType.dSleep(100)
            dType.SetQueuedCmdStopExec(api)
            
            # Rail Setup
            dType.SetDeviceWithL(api, True, version=1)
            dType.SetPTPLParams(api, 99, 99, isQueued=1)
            
            # Mover a posicion de vision
            dType.SetQueuedCmdClear(api)
            lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVJXYZMode, 200, 0, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)[0]
            dType.SetQueuedCmdStartExec(api)
            while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                dType.dSleep(100)
            
            return api
        else:
            print("[ERROR] No se pudo conectar al Dobot. Revisa el cable y el puerto COM3.")
            return None
    except Exception as e:
        print(f"[CRITICAL] Error en setup_dobot: {e}")
        return None

# ================================================================
# 5. BUCLE PRINCIPAL (MAIN)
# ================================================================
def main():
    global global_frame
    
    # 1. Iniciar Flask para que Node-RED pueda ver el video
    threading.Thread(target=run_flask, daemon=True).start()

    # 2. Conectar Dobot
    api = setup_dobot()
    if not api: return

    # 3. Cargar Matriz de Calibración
    matrix_path = os.path.join(script_dir, "calibration_matrix.npy")
    if not os.path.exists(matrix_path):
        print("[ERROR] calibration_matrix.npy no encontrado. Ejecuta la calibración primero.")
        return
    calibration_matrix = np.load(matrix_path)
    
    # 3.5 Cargar Mascara de Vision (Opcional)
    vision_mask_path = os.path.join(script_dir, "vision_mask.npy")
    vision_mask_polygon = None
    if os.path.exists(vision_mask_path):
        vision_mask_polygon = np.load(vision_mask_path)
        print(f"[INFO] Mascara de vision cargada con {len(vision_mask_polygon)} puntos.")
    else:
        print("[INFO] No se encontro vision_mask.npy. Procesando todo el frame.")

    # 4. Iniciar Cámara Hikrobot
    deviceList = MV_CC_DEVICE_INFO_LIST()
    MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, deviceList)
    if deviceList.nDeviceNum == 0:
        print("[ERROR] No se encontró ninguna cámara Hikrobot.")
        return

    cam = MvCamera()
    stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    cam.MV_CC_CreateHandle(stDeviceList)
    cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    
    stParam = MVCC_INTVALUE()
    cam.MV_CC_GetIntValue("PayloadSize", stParam)
    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()
    cam.MV_CC_StartGrabbing()
    stbInfo = MV_FRAME_OUT_INFO_EX()

    print("[SUCCESS] Sistema iniciado. Esperando cubos...")

    stable_color = None
    stable_start_time = 0

    try:
        while True:
            # Captura de frame con byref para evitar crashes
            ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), payload_size, stbInfo, 1000)
            
            if ret == 0:
                nparr = np.frombuffer(data_buf, dtype=np.uint8, count=payload_size)
                
                # Reconstrucción del frame según el tipo de pixel
                if stbInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif stbInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                else:
                    # Intento genérico si el tipo es desconocido
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, -1))

                # Procesamiento de imagen
                hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (11, 11), 0), cv2.COLOR_BGR2HSV)
                
                roi_mask = None
                if vision_mask_polygon is not None:
                    roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(roi_mask, [vision_mask_polygon], 255)
                    
                detected_color, detected_center, max_area = None, None, 0

                for color in ['red', 'blue', 'green', 'yellow']:
                    mask = apply_color_mask(hsv, color)
                    if mask is not None:
                        if roi_mask is not None:
                            mask = cv2.bitwise_and(mask, roi_mask)
                        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        for c in conts:
                            area = cv2.contourArea(c)
                            if area > 1000 and area > max_area:
                                # Shape filter to ensure we only pick up squares
                                peri = cv2.arcLength(c, True)
                                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                                
                                if len(approx) == 4:
                                    M = cv2.moments(c)
                                    if M["m00"] > 0:
                                        max_area, detected_color = area, color
                                        detected_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # Lógica de estabilidad y movimiento
                if detected_center:
                    cv2.circle(frame, detected_center, 15, (0, 255, 0), 3)
                    if stable_color == detected_color:
                        elapsed = time.time() - stable_start_time
                        cv2.putText(frame, f"{stable_color}: {elapsed:.1f}s", (20, 60), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
                        
                        if elapsed >= 2.0:
                            print(f"[ACTION] Procesando cubo {detected_color}...")
                            
                            # Conversión de coordenadas Cámara -> Robot
                            click_pt = np.array([[[detected_center[0], detected_center[1]]]], dtype=np.float32)
                            robot_pt = cv2.perspectiveTransform(click_pt, calibration_matrix)
                            tx, ty = robot_pt[0][0][0], robot_pt[0][0][1]
                            
                            # Configuración de destino según color
                            drop_map = {'red': 950, 'blue': 850, 'green': 750, 'yellow': 650}
                            dr = drop_map.get(detected_color, 650)

                            # Ejecución de comandos del brazo
                            dType.SetQueuedCmdClear(api)
                            # Pick
                            dType.SetPTPWithLCmd(api, 1, tx, ty, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                            dType.SetPTPWithLCmd(api, 1, tx, ty, CUBE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                            dType.SetEndEffectorSuctionCup(api, True, True, isQueued=1)
                            dType.SetWAITCmd(api, 500, isQueued=1)
                            dType.SetPTPWithLCmd(api, 1, tx, ty, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                            # Place
                            dType.SetPTPWithLCmd(api, 1, 0, 200, SAFE_Z, 0, dr, isQueued=1)
                            dType.SetPTPWithLCmd(api, 1, 0, 200, 0, 0, dr, isQueued=1)
                            dType.SetEndEffectorSuctionCup(api, True, False, isQueued=1)
                            dType.SetWAITCmd(api, 500, isQueued=1)
                            # Volver
                            lastIdx = dType.SetPTPWithLCmd(api, 1, 200, 0, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)[0]
                            
                            dType.SetQueuedCmdStartExec(api)
                            while lastIdx > dType.GetQueuedCmdCurrentIndex(api)[0]:
                                dType.dSleep(100)
                            dType.SetQueuedCmdStopExec(api)
                            
                            # Reiniciar detección tras el movimiento
                            stable_start_time = time.time()
                    else:
                        stable_color, stable_start_time = detected_color, time.time()
                else:
                    stable_color = None

                # Actualizar frame global para Flask
                global_frame = cv2.resize(frame, (640, 480))

                # Mostrar ventana local (puede fallar en Node-RED, por eso el try)
                try:
                    cv2.imshow("Vision Dobot", global_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break
                except:
                    pass
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] Detenido por el usuario.")
    except Exception as e:
        print(f"[CRITICAL] Error en bucle principal: {e}")
    finally:
        print("[CLEANUP] Cerrando recursos...")
        cam.MV_CC_StopGrabbing()
        cam.MV_CC_CloseDevice()
        if api:
            dType.DisconnectDobot(api)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()