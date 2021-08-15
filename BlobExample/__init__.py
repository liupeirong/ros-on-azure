import logging

import azure.functions as func
import os
import subprocess

def main(myblob: func.InputStream, outputblob: func.Out[func.InputStream]):
    logging.info(f"function received {myblob.name}, size: {myblob.length} bytes")

    input_folder = '/tmp/input'
    output_folder = '/tmp/output'
    os.makedirs(input_folder, exist_ok=True)
    os.makedirs(output_folder, exist_ok=True)

    file_name = os.path.basename(myblob.name)
    file_name_in = os.path.join(input_folder, file_name)
    file_name_out = os.path.join(output_folder, file_name) + ".txt"

    with open(file_name_in, "wb") as local_file:
        local_file.write(myblob.read())

    output_file = open(file_name_out, "wb")
    subprocess.run([
        f"source /opt/ros/noetic/setup.bash && source /catkin_ws/install_isolated/setup.bash "
        f"&& rosbag reindex {file_name_in} "
        f"&& rosbag info {file_name_in} "
        f"&& cartographer_rosbag_validate -bag_filename {file_name_in}"],
        shell=True, executable="/bin/bash", stdout=output_file, stderr=subprocess.STDOUT)

    output_file = open(file_name_out, "rb")
    outputblob.set(output_file.read())
