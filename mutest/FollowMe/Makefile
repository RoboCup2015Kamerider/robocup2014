#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

OBJS = CheckPoint.so CheckPoint2.so ITP1.so ITP2.so FinishLine.so FollowMeHuman.so FollowMeTestRobot.so Wall.so score.so Operator.so Moderator.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $< `pkg-config --cflags --libs opencv`

clean:
	rm ./*.so
