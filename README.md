# *Y*ET *A*NOTHER *P*HOTO*G*RAMMETRY *T*OOL

Simple photogrammetry tool to recover structure from motion.

# Usage
## Calibrate Camera
`yapgt.exe --calibrationImages ./resources/calibration --calibrateRow 6 --calibrateColumn 9 --savecalibration ./calibration.xml`

This uses the images stored in ./resources/calibration to calibrate the camera. It's important that `calibrateRow` and
`calibrateColumn` are submitted correctly.

## Match Images
`yapgt.exe --loadCalibration ./calibration.xml --matchImages ./resources/box/ --matchOutput ./resources/matches --out ./resources/box.ply`

This loads the camera calibration from the file ./calibration.xml.
It then will proceed to match the images stored in ./resources/box/.
The matched image pairs with visualized keypoints and matches will be stores in the directory ./resources/matches.
The PLY file will be exported to ./resources/box.ply.