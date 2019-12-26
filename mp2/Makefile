all: robotest

robotest: robotest.cc irobot-create.o
	g++ -g -std=c++11 -c robotest.cc 
	g++ -g -std=c++11 -o robotest robotest.o irobot-create.o -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc  -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d

irobot-create.o: irobot-create.cc
	g++ -g -std=c++11 -c irobot-create.cc

clean:
	rm -f robotest *.o
