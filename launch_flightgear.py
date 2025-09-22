import subprocess
import platform
import os

def launch_flightgear():
    fg_root = "C:/Program Files/FlightGear" if platform.system() == "Windows" else "/usr/share/games/flightgear"
    
    cmd = [
        f"{fg_root}/bin/fgfs",
        "--fdm=null",
        "--native-fdm=socket,in,30,,5500,udp",
        "--aircraft=uav",
        "--fg-aircraft=/path/to/your/uav/model",
        "--altitude=1000",
        "--heading=90",
        "--offset-distance=0",
        "--offset-azimuth=0"
    ]
    
    subprocess.Popen(cmd)

if __name__ == "__main__":
    launch_flightgear()