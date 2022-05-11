# Makefile that goes in `catkin_ws`, optimized for compiling with
# Emacs.

all:
	bash -c '. /opt/ros/noetic/setup.bash && catkin_make | sed -r "s/\x1B\[(([0-9]{1,2})?(;)?([0-9]{1,2})?)?[m,K,H,f,J]//g"'

clean:
	bash -c '. /opt/ros/noetic/setup.bash && catkin_make clean | sed -r "s/\x1B\[(([0-9]{1,2})?(;)?([0-9]{1,2})?)?[m,K,H,f,J]//g"'
