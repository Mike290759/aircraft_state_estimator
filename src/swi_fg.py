import time
import math
from flightgear_python.fg_if import FDMConnection, CtrlsConnection

def ctrls_callback(ctrls_data, event_pipe):
    global control_id
    global control_value
    if event_pipe.child_poll():
       (control_id_req, control_value_req,) = event_pipe.child_recv()

       control_id = control_id_req
       control_value = control_value_req

       match control_id:
           case "aileron":
               ctrls_data.aileron = control_value

       return ctrls_data

def ctrls_event_pipe(tx_port, rx_port):
    ctrls_conn = CtrlsConnection()
    ctrls_conn.connect_tx('localhost', tx_port)
    return ctrls_conn.connect_rx('localhost', rx_port, ctrls_callback)

def set_ctrl(control_id, control_value, ctrls_event_pipe):
    ctrls_event_pipe.parent_send((control_id, control_value,))
