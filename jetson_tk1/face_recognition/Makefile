TARGET_PROGRAM          := face_train face_recognition
TARGET_COMPILER         := g++
BUILD_GENERAL_OPTIONS   := -Wall -g -o
BUILD_PKGCONFIG_OPTIONS := `pkg-config --cflags --libs opencv`

all: ${TARGET_PROGRAM}

face_train:
	${TARGET_COMPILER} ${BUILD_GENERAL_OPTIONS} \
	$@ $@.cpp ${BUILD_PKGCONFIG_OPTIONS}

face_recognition:
	${TARGET_COMPILER} ${BUILD_GENERAL_OPTIONS} \
	$@ $@.cpp ${BUILD_PKGCONFIG_OPTIONS}

.PHONY: clean

clean:
	rm ${TARGET_PROGRAM}
