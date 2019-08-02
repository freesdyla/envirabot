import time
import subprocess
import os
import psutil
import smtplib, ssl

working_dir = 'C:\\Users\\lietang123\\Documents\\RoAdFiles\\LineProfilerRobotArmTest\\LineProfilerRobotArmTest'
enviratron_rover_path = "C:\\Users\\lietang123\\Documents\\RoAdFiles\\LineProfilerRobotArmTest\\Release\\Enviratron.exe"
thermo_cam_server_path = 'C:\\Users\\lietang123\\Documents\\RoAdFiles\\FlirThermoCamServer\\FlirThermoCamServer\\bin\\Release\\FlirThermoCamServer.exe'

def notify_managers():
    
    port = 587  # For starttls
    password = 'abetang123'

    # Create a secure SSL context
    context = ssl.create_default_context()

    with smtplib.SMTP("smtp.gmail.com", port) as server:
        server.starttls(context=context)
        server.login("isu.enviratron@gmail.com", password)
        
        # TODO: Send email here
        sender_email = "isu.enviratron@gmail.com"
        receiver_email = ["baoyin89@gmail.com"]
        message = """\
        Subject: Rover stopped

        Remote desktop and check log_file.txt"""

        server.sendmail(sender_email, receiver_email, message)

        
def shutdown_enviratron():
    for pid in (process.pid for process in psutil.process_iter() if process.name()=='FlirThermoCamServer.exe' or process.name() =='Enviratron.exe' ):
        p = psutil.Process(pid)
        p.terminate()

def start_enviratron():
    shutdown_enviratron()
    p = subprocess.Popen(enviratron_rover_path, cwd=working_dir)

    #constantly check if Enviratron.exe is running.
    while p.poll() == None:
        time.sleep(10)

    shutdown_enviratron()
    notify_managers()

start_enviratron()

