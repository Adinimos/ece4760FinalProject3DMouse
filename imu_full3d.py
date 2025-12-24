import win32com.client
import serial
import time
import math

# Configuration
COM_PORT = 'COM3'
BAUD_RATE = 115200
ROTATION_SCALE = 1.0      # sensitivity
NOISE_THRESHOLD = 0.005   # Minimum change to trigger update

# quaternion math functions needed for orientation processing
# Normalize quaternion
def quat_normalize(w, x, y, z):
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 0.0001:
        return 1.0, 0.0, 0.0, 0.0
    return w/norm, x/norm, y/norm, z/norm


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )


def quat_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_to_axis_angle(w, x, y, z):
    w = max(-1.0, min(1.0, w)) # Clamp w to valid range
    angle = 2.0 * math.acos(abs(w))
    
    s = math.sqrt(1.0 - w*w)
    if s < 0.0001:
        return 0.0, 0.0, 1.0, 0.0
    
    # Handle negative w (quaternion double-cover)
    if w < 0:
        x, y, z = -x, -y, -z
    
    return angle, x/s, y/s, z/s


def main():
    print('='*60)
    print('SOLIDWORKS IMU - Full 3-Axis Quaternion Control')
    print('='*60)
    
    # Connect to SolidWorks
    print('Connecting to SolidWorks...')
    try:
        swApp = win32com.client.GetObject(Class='SldWorks.Application')
    except:
        swApp = win32com.client.Dispatch('SldWorks.Application')
    
    model = swApp.ActiveDoc
    if not model:
        print('ERROR: No document open')
        return
    
    view = model.ActiveView
    print('Connected')
    
    # Connect to serial
    print(f'Connecting to {COM_PORT}...')
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1.5)
    ser.reset_input_buffer()
    print('IMU connected')
    
    print('\n' + '='*60)
    print('Move IMU to rotate. Ctrl+C to stop.')
    print('='*60 + '\n')
    
    last_quat = None
    frame = 0
    
    try:
        while True:
            # Drain buffer, keep only latest
            line = None
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                continue
            
            parts = line.split()
            if len(parts) != 4:
                continue
            
            try:
                w, x, y, z = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
            except:
                continue
            
            w, x, y, z = quat_normalize(w, x, y, z)
            current = (w, x, y, z)
            
            if last_quat is None:
                last_quat = current
                continue
            
            # Delta quaternion = current * inverse(last)
            delta = quat_multiply(current, quat_conjugate(last_quat))
            
            # Convert to axis-angle
            angle, ax, ay, az = quat_to_axis_angle(*delta)
            
            if angle < NOISE_THRESHOLD:
                continue
            
            # Map IMU axes to SolidWorks view axes
            # IMU: X=right, Y=forward, Z=up
            # View: X=right, Y=up, Z=out of screen
            # So: IMU_X -> View_X, IMU_Z -> View_Y, IMU_Y -> -View_Z
            
            rot_x = angle * ax * ROTATION_SCALE   # Pitch (tilt forward/back)
            rot_y = angle * az * ROTATION_SCALE   # Yaw (turn left/right) - IMU Z maps to view Y
            rot_z = angle * (-ay) * ROTATION_SCALE  # Roll - IMU Y maps to -view Z
            
            # Apply rotations
            if abs(rot_x) > 0.001 or abs(rot_y) > 0.001:
                view.RotateAboutCenter(rot_x, rot_y)
            
            if abs(rot_z) > 0.001:
                view.RollBy(rot_z)
            
            last_quat = current
            frame += 1
            
            if frame % 100 == 0:
                print(f'{frame}: angle={math.degrees(angle):.1f}degrees axis=[{ax:.2f},{ay:.2f},{az:.2f}]')
                
    except KeyboardInterrupt:
        print(f'\nStopped. {frame} updates.')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
