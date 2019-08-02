
# this script is used to initialize pot positions for a new experiment. Edit pot_diameter, rows and cols accordingly.
# To prevent accidently overwriting the file used by Enviratron program, the new file is named as "new_chamber_pots_configuration.yml"
# Remove "new_" for a new experiment
import cv2
import numpy as np

rows = 2
cols = 7
pot_diameter = 0.2286

path = 'C:\\Users\\lietang123\\Documents\\RoAdFiles\\LineProfilerRobotArmTest\\LineProfilerRobotArmTest\\new_chamber_pots_configuration.yml'

fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)

fs.write('pot_diameters', np.ones( (1,8), np.single)*pot_diameter)
fs.write('pot_position_0', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_1', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_2', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_3', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_4', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_5', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_6', np.zeros( (rows,cols,3), np.single ))
fs.write('pot_position_7', np.zeros( (rows,cols,3), np.single ))

fs.release()

# fix yml header format. Enviratron program uses an older version of OpenCV. This script uses a newer version.
# need to replace the ":" with " " in the first line
file = open(path, mode='r')
all_lines = file.read()
file.close()
new_lines = all_lines.replace(':', ' ', 1)
#print(new_lines)
file = open(path, mode='w')
file.write(new_lines)
file.close()

