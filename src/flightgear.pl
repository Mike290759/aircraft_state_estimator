/* Part of aircraft_state_estimator

Author:        Mike Elston
E-mail:        mike.elston@gmail.com

MIT License

Copyright (c) 2025 Mike Elston

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

:- module(flightgear,
          [
          ]).

/** <module> FlightGear interface

Uses the "generic" protocol to specify the format of data exchanged
between Prolog and FlightGear Flight Simulator. See
https://wiki.flightgear.org/Generic_protocol#Input/Output_Parameters

Put generic_test.xml in	/home/mike/.fgfs/fgdata_2024_1/Protocol

Initally used the Python interface to FlightGear via Janus but this
proved complex and an issue with an uninitialised stdout stream on the
sub-process used for the socket interface was never resolved.
*/

:- use_module(library(socket)).
:- use_module(library('plot/plotter')).
:- use_module(library('plot/axis')).
:- use_module(library(autowin)).
:- use_module(library(lists)).
:- use_module(library(pce)).
:- use_module(library(process)).
:- use_module(library(readutil)).


%!  udp is det.
%
%   The protocol definition file is FG_ROOT/Protocol.swi_fg.xml. A
%   symbolic link pointing to src/swi_fg.xml is used to avoid
%   duplicating the file.
%
%   @todo Make the polling frequency a fact

user:udp :-
    fg_root(FG_ROOT),
    format(string(FG_ROOT_Arg), '--fg-root=~w', [FG_ROOT]),
    swi_fg_ports(TX_Port, RX_Port),
    format(string(Generic_TX_Arg), '--generic=socket,out,1,localhost,~w,udp,swi_fg', [TX_Port]),
    format(string(Generic_RX_Arg), '--generic=socket,in,1,localhost,~w,udp,swi_fg', [RX_Port]),
     process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   [FG_ROOT_Arg, Generic_TX_Arg, Generic_RX_Arg,
                    '--airport=NZWN',
                    '--runway=34'],
                   [stderr(pipe(Out))]),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    thread_create(log_state(TX_Port), _, [detached(true)]),
    udp_socket(Socket),
    repeat,
    member(Rudder, [-1.0, 0.0, +1.0]),
    format(string(Payload), '~q~n', [Rudder]),
    udp_send(Socket, Payload, localhost:RX_Port, [as(string)]),
    sleep(1),
    fail.


%    thread_create(fly_heading(340), _, [detached(true)]),
%    thread_create(fly_pitch(4), _, [detached(true)]).


%!  log_state(+Port) is det.

log_state(Port) :-
    udp_socket(Socket),
    tcp_bind(Socket, Port),
    repeat,
    udp_receive(Socket, Data, _, [as(term)]),
    format('~q~n', [Data]),
    fail.


%! swi_fg_ports(-TX_Port, -RX_Port) is det.
%
%   Port directions are from the point of view on FlightGear

swi_fg_ports(5501, 5502).


%!  fg_root(-Dir) is det.
%
%  Location of FlightGear main data directory

fg_root('/home/mike/.fgfs/fgdata_2024_1').


%! steer_heading_on_ground(+Plotter) is det.

steer_heading_on_ground(Plotter) :-
    P = 0.03,
    I = 0.03,
    D = 0.0,
    Control_Min = -0.5,
    Control_Max = +0.5,
    flightgear_http_connection(HTTP_Conn),
    pid_controller(on_ground(HTTP_Conn),                                                               % Guard
                   required_heading,                                                                   % Setpoint
                   get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn),    % State_Value
                   set_prop('/controls/flight/rudder', HTTP_Conn),                                     % Control
                   direction_difference,                                                               % Error calculation
                   P,
                   I,
                   D,
                   Control_Min,
                   Control_Max,
                   Plotter,
                   0.1,
                   green).


%!  on_ground(+HTTP_Conn) is semidet.

on_ground(HTTP_Conn) :-
    get_prop('/position/altitude-agl-ft', HTTP_Conn, Altitude_AGL_Feet),
    Altitude_AGL_Feet < 5.


%!  required_heading(-Heading) is det.

required_heading(338).


%!  fly_heading is det.

fly_heading(Plotter) :-
    P = 0.01,
    I = 0.0,
    D = 0.0,
    Control_Min = -0.5,
    Control_Max = +0.5,
    flightgear_http_connection(HTTP_Conn),
    pid_controller(\+ on_ground(HTTP_Conn),                                                            % Guard
                   required_heading,                                                                   % Setpoint
                   get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn),    % State_Value
                   set_prop('/controls/flight/aileron', HTTP_Conn),                                    % Control
                   direction_difference,                                                               % Error calculation
                   P,
                   I,
                   D,
                   Control_Min,
                   Control_Max,
                   Plotter,
                   0.1,
                   red).



%!  fly_pitch(+Required_Pitch_Deg) is det.
%
%   Adjust trim to fly a specified pitch angle
%
%   Positive elevator-trim puts nose down

fly_pitch(Required_Pitch_Deg) :-
    flightgear_http_connection(HTTP_Conn),
    Sample_Time = 0.2,
    repeat,
    sleep(Sample_Time),
    get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', HTTP_Conn, Indicated_Speed_KT),
    get_prop('/instrumentation/attitude-indicator/indicated-pitch-deg', HTTP_Conn, Indicated_Pitch_Deg),
    writeln(ias(Indicated_Speed_KT)),
    (   Indicated_Speed_KT < 50
    ->  Required_Pitch_Deg_1 = 4
    ;   Required_Pitch_Deg_1 = Required_Pitch_Deg
    ),
    Pitch_Error is Indicated_Pitch_Deg - Required_Pitch_Deg_1,   % +ve if pitch too high
    clamped(0.3 * Pitch_Error * Sample_Time, -0.5, +0.5, New_Elevator_Trim),
    writeln(fly_pitch(pitch(Indicated_Pitch_Deg), pitch_error(Pitch_Error), elevator_trim(New_Elevator_Trim))),
    set_prop('/controls/flight/elevator-trim', HTTP_Conn, New_Elevator_Trim),
    fail.


%!  align_heading_indicator is det.
%
%   Align the DI with the magnetic compass

align_heading_indicator :-
    flightgear_http_connection(HTTP_Conn),
    get_prop('/instrumentation/magnetic-compass/indicated-heading-deg', HTTP_Conn, Magnetic_Compass_Indicated_Heading),
    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn, Unaligned_Indicated_Heading),
    get_prop('/instrumentation/heading-indicator/align-deg', HTTP_Conn, Initial_Align_Deg),
    direction_difference(Magnetic_Compass_Indicated_Heading, Unaligned_Indicated_Heading, Heading_Indicator_Alignment_Error),
    Align_Deg is integer(Initial_Align_Deg + Heading_Indicator_Alignment_Error) mod 360,
    set_prop('/instrumentation/heading-indicator/align-deg', HTTP_Conn, Align_Deg).


%!  pid_controller(:Guard, :Setpoint_Pred, :State_Value_Pred,
%!                 :Control_Pred, :Error_Pred, +P, +I, +D, +Control_Min,
%!                 +Control_Max, +Plotter, +Plot_Scale_Y, +Plot_Colour)
%!                 is det.
%
%   PID controller to reduce the mismatch between the input setpoint
%   and the desired state value. This predicate runs forever on its own
%   thread.
%
%   @arg Guard run PID loop iff Guard succeeds
%   @arg Setpoint_Pred name of goal to get the desired set point.
%   e.g if Setpoint_Pred was required_roll_angle,
%   required_roll_angle(-X) will be called.
%
%   @arg State_Value_Pred name of goal to get the current state value e.g if
%   State_Value was roll_angle, roll_angle(X) will be called.
%
%   @arg Control_Pred name of goal to apply the output of the PID
%   controller e.g. if Control_Output was set_aileron, set_aileron(X)
%   will be called.
%
%   @arg Error_Pred name of goal to calculate the error from the
%   Setpoint and the State_Value e.g. if Error_Pred was
%   direction_difference, direction_difference(Setpoint, State_Value,
%   Error) would be called
%
%   @arg P float the proportional gain
%   @arg I float the integration gain
%   @arg D float the derivative gain
%   @arg Control_Min float
%   @arg Control_Max float
%   @arg Plotter graph object
%   @arg Plot_Scale_Y factor to scale the error for plotting
%   @arg Plot_Colour colour of the plot line

:- meta_predicate
    pid_controller(:, :, :, :, :, +, +, +, +, +, +, +, +).

pid_controller(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Control_Min, Control_Max, Plotter, Plot_Scale_Y, Plot_Colour) :-
    send(Plotter, graph, new(Plot_Graph, plot_graph)),
    send(Plot_Graph, colour, Plot_Colour),
    call(State_Value_Pred, State_Value),
    sample_time(Sample_Time),
    thread_create(pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Control_Min, Control_Max, Plot_Graph, Plot_Scale_Y, Sample_Time, 0, State_Value, 0),
                  _,
                  [detached(true)]).


%! pid_controller_1(:Setpoint_Pred, :State_Value_Pred, :Control_Pred,
%!                  :Error_Pred, +P, +I, +D, +Control_Min, +Control_Max,
%!                  +Plot_Graph, +Plot_Scale_Y, +Sample_Time, +Error_Sum,
%!                  +Previous_Error, +Elapsed_Time) is det.

:- meta_predicate
    pid_controller_1(:, :, :, :, :, +, +, +, +, +, +, +, +, +, +, +).

pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Control_Min, Control_Max, Plot_Graph, Plot_Scale_Y, Sample_Time, Error_Sum, Previous_Error, Elapsed_Time) :-
    (   Guard
    ->  call(Setpoint_Pred, Setpoint),
        call(State_Value_Pred, State_Value),
        call(Error_Pred, Setpoint, State_Value, Error),
        Graph_Y is Error * Plot_Scale_Y,
        send(Plot_Graph, append, Elapsed_Time, Graph_Y),
        Error_Sum_1 is Error_Sum + Error,
        clamped(P * Error + I * Error_Sum * Sample_Time + D * (Error - Previous_Error) / Sample_Time, Control_Min, Control_Max, Control),
        call(Control_Pred, Control)
    ;   Error_Sum_1 = Error_Sum,
        Error = Previous_Error
    ),
    Elapsed_Time_1 is Elapsed_Time + Sample_Time,
    sleep(Sample_Time),
    !,   % Precautionary green cut
    pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Control_Min, Control_Max, Plot_Graph, Plot_Scale_Y, Sample_Time, Error_Sum_1, Error, Elapsed_Time_1).


%!  sample_time(-Sample_Time) is det.
%
%   @arg Sample_Time float sample time in seconds

sample_time(0.2).



%!  direction_difference(+H1, +H2, -Diff) is det.
%
%   @arg Diff [-180, +180]

direction_difference(H1, H2, Diff) :-
    Diff is integer((H1 - H2 + 540)) mod 360 - 180.


%!   clamped(+Expr, +Left, +Right, -Clamped) is det.

clamped(Expr, Left, Right, Clamped) :-
    X is Expr,
    (   X =< Left
    ->  Clamped is Left
    ;   X >= Right
    ->  Clamped is Right
    ;   Clamped = X
    ).


%!  pid_plotter(-P) is det.

pid_plotter(P) :-
    new(W, auto_sized_picture('PID')),
    send(W, max_size, size(1800, 600)),
    send(W, display, new(P, plotter)),
    send(P, axis, plot_axis(x, 0, 200, @(default), 1500)),
    send(P, axis, plot_axis(y, -1, 1, @(default), 400)),
    send(W, open).






