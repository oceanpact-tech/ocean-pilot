import socket
import time

HOST = '0.0.0.0'  
PORT = 8081

PAYLOAD = '{"droneName":"drone_1","speed":{"x":2.8,"y":0.1,"z":0},"heading":69.3,"attitude":{"pitch":1.1,"roll":-7.6,"yaw":69.3},"location":{"latitude":-22.63020595490781,"longitude":-43.020841454394805,"altitude":42.800000000000004},"lrfTarget":null,"phoneLocation":{"latitude":0.0,"longitude":0.0,"heading":113.70138047163688,"pressure":0.0,"battery":91,"wifiRssi":-46},"webRtc":{"sourceWidth":1920,"sourceHeight":1080,"outputWidth":1920,"outputHeight":1080,"requestedWidth":0,"requestedHeight":0,"targetFps":10,"inputFps":30.39770277460798,"outputFps":6.86399740071793,"droppedFps":23.533705373890047,"averageFrameProcessingMs":0.107157,"totalFrames":148,"totalDroppedFrames":405,"processingErrors":0,"observerCount":1,"activeCamera":"LEFT_OR_MAIN","status":"running","configuredFps":10,"saturationState":"ok","scaleMode":"native","recoveryCount":0,"lastError":null},"gimbalAttitude":{"pitch":0.5,"roll":0,"yaw":171},"gimbalJointAttitude":{"pitch":1.2000000000000002,"roll":2.3000000000000003,"yaw":90.2},"zoomFl":0,"hybridFl":-1,"opticalFl":-1,"zoomRatio":null,"batteryLevel":17,"satelliteCount":15,"homeLocation":{"latitude":-22.62956500757222,"longitude":-43.02296299800662},"distanceToHome":229.10959903363602,"waypointReached":false,"waypointSeq":0,"intermediaryWaypointReached":false,"yawReached":false,"yawSeq":0,"altitudeReached":false,"altitudeSeq":0,"isRecording":false,"homeSet":false,"remainingFlightTime":1249,"timeNeededToGoHome":48,"timeNeededToLand":40,"totalTime":88,"maxRadiusCanFlyAndGoHome":0.0,"remainingCharge":17,"batteryNeededToLand":9,"batteryNeededToGoHome":18,"seriousLowBatteryThreshold":10,"lowBatteryThreshold":30,"flightMode":"GPS_NORMAL","readyToTakeoff":false,"takeoffBlockReason":"LOW_POWER","isManualOverrideActive":true,"autoSensingActive":false,"detectedTargets":[]}\n'

def start_simulator():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        
        print(f"[Simulator] Waiting for connection at {PORT}...")
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"[Simulator] Client connected: {addr}")
                try:
                    while True:
                        conn.sendall(PAYLOAD.encode('utf-8'))
                        time.sleep(0.05)  # Send telemetry data every 50ms
                        
                except (ConnectionResetError, BrokenPipeError):
                    print(f"[Simulator] Client {addr} disconnected.")

if __name__ == "__main__":
    try:
        start_simulator()
    except KeyboardInterrupt:
        print("\n[Simulator] Stopped by user.")