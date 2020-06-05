cd ~/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/src/acado_diff/

mkdir build_python2
python setup.py build --build-base=build_python2/
cp -i build_python2/lib.linux-x86_64-2.7/acado.so ~/Documentos/simulation/catkin_ws/devel/lib/python2.7/dist-packages
