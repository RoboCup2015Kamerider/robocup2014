#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

#オブジェクトファイルの指定
#OBJS     = Test.so TrashBox.so Camera.so Wagon.so Moderator.so
OBJS     = CleanUpDemo2014Robo.so TrashBox.so Wagon.so Moderator.so
#OBJS     = CleanUpRoboDemo.so TrashBox.so Camera.so Wagon.so Moderator.so
#OBJS     = Test.so TrashBox.so Furniture.so Camera.so Wagon.so Moderator.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $< `pkg-config --cflags --libs opencv`

clean:
	rm ./*.so
