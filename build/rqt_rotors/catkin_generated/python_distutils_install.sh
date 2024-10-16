#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/chris/catkin_ws/src/CrazyS/rqt_rotors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/chris/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/chris/catkin_ws/install/lib/python3/dist-packages:/home/chris/catkin_ws/build/rqt_rotors/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/chris/catkin_ws/build/rqt_rotors" \
    "/usr/bin/python3" \
    "/home/chris/catkin_ws/src/CrazyS/rqt_rotors/setup.py" \
     \
    build --build-base "/home/chris/catkin_ws/build/rqt_rotors" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/chris/catkin_ws/install" --install-scripts="/home/chris/catkin_ws/install/bin"
