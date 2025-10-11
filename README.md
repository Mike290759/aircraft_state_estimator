# FlightGear
https://flightgear-python.readthedocs.io/en/latest/

FlightGear simulator is an AppImage (like an EXE on Windows). Download it and move it to ~/Applications

# flightgear-python Installation
    cd ~/aircraft_state_estimator

    python -m venv flightgear

    source flightgear/bin/activate
    pip3 install flightgear-python
    ...
    deactivate

# Demos
## Aircraft rolling and climbing

In a Terminal window:

    ~/Applications/flightgear-2024.1.2-linux-amd64.AppImage --native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp --fdm=null --airport=NZPP

Select view  with v

In another terminal window:

    cd ~/aircraft_state_estimator
    source flightgear/bin/activate
    python src/simple_fdm.py

## Wing Leveler
In a Terminal window:

    ~/Applications/flightgear-2024.1.2-linux-amd64.AppImage --native-fdm=socket,out,30,localhost,5501,udp --native-ctrls=socket,out,30,localhost,5503,udp --native-ctrls=socket,in,30,localhost,5504,udp

In another terminal window:

    cd ~/aircraft_state_estimator
    source flightgear/bin/activate
    python src/simple_wing_leveler.py


# SWIPL
    cd ~/aircraft_state_estimator
    swipl-win load.pl
    test.
