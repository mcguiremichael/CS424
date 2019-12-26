all: robovision

robovision: robovision_FLAN.cc
	g++ -std=c++11 -o robovision robovision_FLAN.cc -L/opt/vc/lib -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs

clean:
	rm -f robovision
