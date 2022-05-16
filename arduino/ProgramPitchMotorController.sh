# run this shell script to compile and upload the latest PitchMotorController code to the connected Arduino 
arduino-cli compile --fqbn arduino:avr:leonardo PitchMotorController
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:leonardo PitchMotorController/PitchMotorController.ino -v
