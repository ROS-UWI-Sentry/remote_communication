# remote_communication

[Created by following this tutorial.](https://github.com/NVIDIA/jetson-gpio#installation)
Note the inclusion of Robot web tools in the html file to allow communication with ROSBRidge.


The remote's web files are stored in remote_server. 
The browser_reports.py script is used by the remote to store and retrieve santization time and date data. This script uses the reports.txt file for this. It uses the file path that depends on the user's name so relative addressing is a future upgrade.
The heartbeat_jetson_remote.py script is used by the Jetson to ask the remote if it is connected. If not this script publishes the shutdown message.

See the main_control README for more details.
