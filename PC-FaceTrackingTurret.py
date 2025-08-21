import cv2
import mediapipe as mp
import serial
import requests
import sys
import time
import asyncio
from collections import deque
import numpy as np
from datetime import datetime

# PIDController class
class PIDController:
    def __init__(self, p, i, d, min_out=-100.0, max_out=100.0):
        self.kp = p
        self.ki = i
        self.kd = d
        self.min_output = min_out
        self.max_output = max_out
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_update = True

    def _clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        p_term = self.kp * error
        self.integral = self._clamp(self.integral + error * dt, -100.0, 100.0)
        i_term = self.ki * self.integral
        if self.first_update:
            d_term = 0.0
            self.first_update = False
        else:
            d_term = self.kd * (error - self.previous_error) / dt
        self.previous_error = error
        output = self._clamp(p_term + i_term + d_term, self.min_output, self.max_output)
        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_update = True

# ESP32RobotController class (serial communication)
class ESP32RobotController:
    def __init__(self, port="COM13", baudrate=115200, timeout=1, max_retries=5, reconnect_delay=3):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_retries = max_retries
        self.reconnect_delay = reconnect_delay
        self.max_reconnect_delay = 30
        self.serial = None

    async def connect(self):
        retries = 0
        delay = self.reconnect_delay
        while retries < self.max_retries:
            try:
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Connected to ESP32 at {self.port}")
                return True
            except serial.SerialException as e:
                retries += 1
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Serial connection attempt {retries}/{self.max_retries} to {self.port} failed: {e}")
                if retries < self.max_retries:
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, self.max_reconnect_delay)
        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Failed to connect to ESP32 at {self.port} after {self.max_retries} attempts")
        return False

    async def disconnect(self):
        if self.serial:
            try:
                self.serial.close()
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Disconnected from ESP32")
            except serial.SerialException as e:
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error during disconnection: {e}")
            finally:
                self.serial = None

    async def set_pan_angle(self, pan):
        """Send pan angle to ESP32 in 'pan:X' format with reconnection on failure."""
        if not self.serial:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] No serial connection. Attempting to reconnect...")
            if not await self.connect():
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping pan command.")
                return False
        command = f"pan:{int(pan)}\n".encode()
        try:
            self.serial.write(command)
            self.serial.flush()
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent pan command: {command.decode().strip()}")
            return True
        except serial.SerialException as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Failed to send pan command: {e}")
            await self.disconnect()
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Attempting to reconnect...")
            if await self.connect():
                try:
                    self.serial.write(command)
                    self.serial.flush()
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent pan command after reconnect: {command.decode().strip()}")
                    return True
                except serial.SerialException as e:
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Retry failed after reconnect: {e}")
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping pan command.")
            return False

    async def set_tilt_angle(self, tilt):
        """Send tilt angle to ESP32 in 'tilt:Y' format with reconnection on failure."""
        if not self.serial:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] No serial connection. Attempting to reconnect...")
            if not await self.connect():
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping tilt command.")
                return False
        command = f"tilt:{int(tilt)}\n".encode()
        try:
            self.serial.write(command)
            self.serial.flush()
            # print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent tilt command: {command.decode().strip()}")
            return True
        except serial.SerialException as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Failed to send tilt command: {e}")
            await self.disconnect()
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Attempting to reconnect...")
            if await self.connect():
                try:
                    self.serial.write(command)
                    self.serial.flush()
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent tilt command after reconnect: {command.decode().strip()}")
                    return True
                except serial.SerialException as e:
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Retry failed after reconnect: {e}")
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping tilt command.")
            return False

    async def set_deadband(self, deadband_zone):
        """Send deadzone command to ESP32 with reconnection on failure."""
        if not self.serial:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] No serial connection. Attempting to reconnect...")
            if not await self.connect():
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping deadband command.")
                return False
        if deadband_zone:
            command = "deadzone\n".encode()
            try:
                self.serial.write(command)
                self.serial.flush()
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent deadband command: {command.decode().strip()}")
                return True
            except serial.SerialException as e:
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Failed to send deadband command: {e}")
                await self.disconnect()
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Attempting to reconnect...")
                if await self.connect():
                    try:
                        self.serial.write(command)
                        self.serial.flush()
                        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Sent deadband command after reconnect: {command.decode().strip()}")
                        return True
                    except serial.SerialException as e:
                        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Retry failed after reconnect: {e}")
                else:
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnection failed. Skipping deadband command.")
                return False

# FaceTrackingSystem class
class FaceTrackingSystem:
    def __init__(self, stream_url, frame_width=1600, frame_height=1200, dead_zone=8.0):  # Increased dead zone
        self.URL = "http://192.168.137.148"
        self.STREAM_URL = stream_url
        self.FRAME_WIDTH = frame_width
        self.FRAME_HEIGHT = frame_height
        self.DEAD_ZONE = dead_zone
        self.MIN_DT = 0.001
        self.SERVO_UPDATE_RATE = 0.2
        self.MAX_ANGLE_CHANGE = 5.0  # Reduced for smoother transitions
        self.ALPHA = 0.3  # Smoothing factor for EMA (0 < ALPHA < 1, lower means more smoothing)
        self.last_servo_update_time = 0.0
        self.max_stream_retries = 5
        self.stream_reconnect_delay = 3
        self.max_stream_reconnect_delay = 30
        self.FRAME_SKIP = 1
        self.frame_counter = 0
        self.last_detection = None
        self.last_target_center = None
        self.in_dead_zone_pan = True
        self.in_dead_zone_tilt = True
        self.last_sent_pan = 90
        self.last_sent_tilt = 90
        self.smoothed_pan = 90  # Initialize smoothed angles
        self.smoothed_tilt = 90

        # Initialize MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

        # OpenCV setup for ESP32-CAM stream
        self.cap = cv2.VideoCapture(self.STREAM_URL)

        # Dynamically set frame dimensions
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.FRAME_HEIGHT, self.FRAME_WIDTH, _ = frame.shape
                self.FRAME_CENTER = (self.FRAME_WIDTH // 2, self.FRAME_HEIGHT // 2)
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Warning: Could not read initial frame to set dimensions")
        else:
            self.FRAME_CENTER = (frame_width // 2, frame_height // 2)

        self.pid_pan = PIDController(p=0.001, i=0.1, d=2, min_out=-90, max_out=90)
        self.pid_tilt = PIDController(p=0.001, i=0.1, d=2, min_out=-90, max_out=90)

        self.servo_pan = 90
        self.servo_tilt = 90
        self.previous_time = time.perf_counter()

        # Parameters for FPS calculation
        self.FPS_WINDOW = 10
        self.fps_queue = deque(maxlen=self.FPS_WINDOW)

        # Log File setup
        self.LOG_FILE = "fps_log.txt"
        self.log_enabled = True
        self.log_flush_interval = 10
        self.log_write_count = 0

    def set_resolution(self, url: str, index: int = 1, verbose: bool = False):
        try:
            if verbose:
                resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
                print("available resolutions\n{}".format(resolutions))
            if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
                requests.get(url + "/control?var=framesize&val={}".format(index))
            else:
                print("Wrong index")
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] SET_RESOLUTION: something went wrong: {e}")

    def set_quality(self, url: str, value: int = 1, verbose: bool = False):
        try:
            if value >= 10 and value <= 63:
                requests.get(url + "/control?var=quality&val={}".format(value))
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] SET_QUALITY: something went wrong: {e}")

    def set_awb(self, url: str, awb: int = 1):
        try:
            awb = not awb
            requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0))
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] SET_AWB: something went wrong: {e}")
        return awb

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def interpolate_angle(self, current_angle, last_sent_angle):
        """Interpolate and smooth servo angle changes using EMA for smoother transitions."""
        max_step = self.MAX_ANGLE_CHANGE
        # Apply EMA for smoothing
        smoothed_angle = self.ALPHA * current_angle + (1 - self.ALPHA) * last_sent_angle
        # Clamp to ensure no sudden jumps
        new_angle = self.clamp(smoothed_angle, last_sent_angle - max_step, last_sent_angle + max_step)
        return new_angle

    async def reconnect_stream(self):
        retries = 0
        delay = self.stream_reconnect_delay
        self.cap.release()
        while retries < self.max_stream_retries:
            try:
                self.cap = cv2.VideoCapture(self.STREAM_URL)
                if self.cap.isOpened():
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Reconnected to ESP32-CAM stream at {self.STREAM_URL}")
                    ret, frame = self.cap.read()
                    if ret:
                        self.FRAME_HEIGHT, self.FRAME_WIDTH, _ = frame.shape
                        self.FRAME_CENTER = (self.FRAME_WIDTH // 2, self.FRAME_HEIGHT // 2)
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    return True
                else:
                    self.cap.release()
                    retries += 1
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Stream reconnection attempt {retries}/{self.max_stream_retries} failed")
                    if retries < self.max_stream_retries:
                        await asyncio.sleep(delay)
                        delay = min(delay * 2, self.max_stream_reconnect_delay)
            except Exception as e:
                retries += 1
                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Stream reconnection attempt {retries}/{self.max_stream_retries} failed: {e}")
                if retries < self.max_stream_retries:
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, self.max_stream_reconnect_delay)
        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Failed to reconnect to ESP32-CAM stream at {self.STREAM_URL} after {self.max_stream_retries} attempts")
        return False

    async def run(self, robot):
        AWB = 1
        self.set_resolution(self.URL, index=10)

        if not self.cap.isOpened():
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error: Could not open video stream from ESP32-CAM at {self.STREAM_URL}")
            await robot.disconnect()
            sys.exit(1)

        try:
            with open(self.LOG_FILE, "a") as log_file:
                if log_file.tell() == 0:
                    log_file.write("Timestamp,FPS\n")

                while True:
                    start_time = time.perf_counter()
                    try:
                        ret, frame = self.cap.read()
                    except Exception as e:
                        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error reading frame from ESP32-CAM: {e}")
                        ret = False

                    current_time = time.perf_counter()
                    dt = max(current_time - self.previous_time, self.MIN_DT)

                    if not ret:
                        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error: Couldn't read frame from ESP32-CAM")
                        if not await self.reconnect_stream():
                            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Exiting due to repeated stream connection failures")
                            break
                        self.previous_time = time.perf_counter()
                        continue

                    # Flip the frame horizontally (left to right)
                    frame = cv2.flip(frame, 1)

                    # Get frame dimensions
                    self.FRAME_HEIGHT, self.FRAME_WIDTH, _ = frame.shape
                    self.FRAME_CENTER = (self.FRAME_WIDTH // 2, self.FRAME_HEIGHT // 2)

                    self.frame_counter += 1
                    process_frame = self.frame_counter % self.FRAME_SKIP == 0

                    if process_frame:
                        # Process frame: face detection and PID control
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        results = self.face_detection.process(rgb_frame)
                        frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

                        # Face Tracking Logic
                        largest_area = 0
                        target_center = None
                        largest_detection = None

                        if results.detections:
                            h, w, _ = frame.shape
                            for detection in results.detections:
                                bbox = detection.location_data.relative_bounding_box
                                x = int(bbox.xmin * w)
                                y = int(bbox.ymin * h)
                                width = int(bbox.width * w)
                                height = int(bbox.height * h)
                                area = width * height

                                if area > largest_area:
                                    largest_area = area
                                    cx = x + width // 2
                                    cy = y + height // 2
                                    target_center = (cx, cy)
                                    largest_detection = (detection, (x, y, width, height))

                            if largest_detection:
                                self.last_detection = largest_detection
                                self.last_target_center = target_center
                            else:
                                self.last_detection = None
                                self.last_target_center = None
                        else:
                            self.last_detection = None
                            self.last_target_center = None

                        # PID Control
                        if self.last_target_center:
                            error_x = self.FRAME_CENTER[0] - self.last_target_center[0]
                            error_y = self.FRAME_CENTER[1] - self.last_target_center[1]

                            # Normalize errors based on ESP32-CAM FOV
                            normalized_error_x = (error_x / self.FRAME_WIDTH) * 100
                            normalized_error_y = (error_y / self.FRAME_HEIGHT) * 100

                            # Update pan angle if outside dead zone
                            if abs(normalized_error_x) >= self.DEAD_ZONE:
                                pan_correction = self.pid_pan.compute(normalized_error_x, dt)
                                new_pan = self.clamp(90 - pan_correction, 0, 180)
                                self.servo_pan = new_pan
                                self.in_dead_zone_pan = False
                            else:
                                self.in_dead_zone_pan = True
                                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Pan in dead zone: {self.servo_pan:.2f} degrees")

                            # Update tilt angle if outside dead zone
                            if abs(normalized_error_y) >= self.DEAD_ZONE:
                                tilt_correction = self.pid_tilt.compute(normalized_error_y, dt)
                                new_tilt = self.clamp(90 - tilt_correction, 0, 170)
                                self.servo_tilt = new_tilt
                                self.in_dead_zone_tilt = False
                            else:
                                self.in_dead_zone_tilt = True
                                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Tilt in dead zone: {self.servo_tilt:.2f} degrees")

                    # Draw visual elements on all frames
                    cv2.line(frame, (self.FRAME_CENTER[0], 0), (self.FRAME_CENTER[0], self.FRAME_HEIGHT), (255, 0, 0), 1)
                    cv2.line(frame, (0, self.FRAME_CENTER[1]), (self.FRAME_WIDTH, self.FRAME_CENTER[1]), (255, 0, 0), 1)

                    if self.last_detection:
                        detection, (x, y, width, height) = self.last_detection
                        self.mp_drawing.draw_detection(frame, detection)
                        cv2.putText(frame, 'Face', (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        cv2.circle(frame, self.last_target_center, 5, (0, 255, 0), -1)
                    cv2.circle(frame, self.FRAME_CENTER, 5, (0, 0, 255), -1)

                    # FPS Calculation
                    elapsed_time = time.perf_counter() - start_time
                    self.fps_queue.append(elapsed_time)
                    avg_elapsed_time = sum(self.fps_queue) / len(self.fps_queue) if self.fps_queue else 1.0
                    fps = 1.0 / avg_elapsed_time if avg_elapsed_time > 0 else 0.0

                    # Logging with buffering
                    if self.log_enabled and log_file:
                        try:
                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            log_file.write(f"{timestamp},{fps:.2f}\n")
                            self.log_write_count += 1
                            if self.log_write_count >= self.log_flush_interval:
                                log_file.flush()
                                self.log_write_count = 0
                        except IOError as e:
                            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error writing to log file: {e}")
                            self.log_enabled = False

                    # Display FPS, marked as Skipped if not processed
                    fps_text = f'FPS: {fps:.2f}' if process_frame else f'FPS: {fps:.2f} (Skipped)'
                    fps_color = (0, 255, 0) if process_frame else (0, 255, 255)
                    cv2.putText(frame, fps_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, fps_color, 2)

                    cv2.imshow("ESP32-CAM Face Tracking", frame)
                    await asyncio.sleep(0)

                    # Update servo angles independently if not in dead zone and enough time has elapsed
                    if current_time - self.last_servo_update_time >= self.SERVO_UPDATE_RATE:
                        if not self.in_dead_zone_pan:
                            interpolated_pan = self.interpolate_angle(self.servo_pan, self.smoothed_pan)
                            self.smoothed_pan = interpolated_pan  # Update smoothed value
                            await robot.set_pan_angle(interpolated_pan)
                            self.last_sent_pan = interpolated_pan
                            # print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Updated pan angle: {interpolated_pan:.2f}°")
                        if not self.in_dead_zone_tilt:
                            interpolated_tilt = self.interpolate_angle(self.servo_tilt, self.smoothed_tilt)
                            self.smoothed_tilt = interpolated_tilt  # Update smoothed value
                            await robot.set_tilt_angle(interpolated_tilt)
                            self.last_sent_tilt = interpolated_tilt
                            # print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Updated tilt angle: {interpolated_tilt:.2f}°")
                        if self.in_dead_zone_pan and self.in_dead_zone_tilt:
                            await robot.set_deadband(True)
                        self.last_servo_update_time = current_time

                    key = cv2.waitKey(1)

                    if key == ord('r'):
                        try:
                            idx = int(input("Select resolution index: "))
                            self.set_resolution(self.URL, index=idx, verbose=True)
                        except ValueError:
                            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Invalid resolution index")
                    elif key == ord('q'):
                        try:
                            val = int(input("Set quality (10 - 63): "))
                            if 10 <= val <= 63:
                                self.set_quality(self.URL, value=val)
                            else:
                                print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Quality must be between 10 and 63")
                        except ValueError:
                            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Invalid quality value")
                    elif key == ord('a'):
                        AWB = self.set_awb(self.URL, AWB)
                    elif key == 27 or key == ord('x'):
                        break

                    self.previous_time = current_time

        except KeyboardInterrupt:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Program terminated by user")
        except IOError as e:
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Error with log file: {e}")
            self.log_enabled = False
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            self.face_detection.close()
            await robot.disconnect()
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Program ended.")

async def main():
    loop = asyncio.get_event_loop()
    try:
        robot = ESP32RobotController(port="COM13", baudrate=115200, timeout=1, max_retries=5)
        await robot.connect()
        tracker = FaceTrackingSystem(stream_url="http://192.168.137.148:81/stream")
        await tracker.run(robot)
    finally:
        if not loop.is_closed():
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.close()

if __name__ == '__main__':
    asyncio.run(main())