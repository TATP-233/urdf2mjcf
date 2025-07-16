import os
import argparse
import xml.dom.minidom as minidom

argparser = argparse.ArgumentParser(description="Update URDF file with new mesh paths.")
argparser.add_argument("urdf_path", type=str, help="Path to the URDF file to be updated.")
argparser.add_argument("-o", "--output", type=str, help="Path to the new URDF file to be updated.", default=None)
args = argparser.parse_args()

# Parse the URDF file
urdf_path = args.urdf_path
if args.output is None:
    new_urdf_path = urdf_path.replace(".urdf", "_new.urdf")
else:
    new_urdf_path = args.output

xmlDoc = minidom.parse(urdf_path)

# Save the modified URDF file
if os.path.exists(new_urdf_path):
    os.remove(new_urdf_path)
print("new_urdf_path = ", new_urdf_path)

with open(new_urdf_path, 'w') as fp:
    xmlDoc.writexml(fp)