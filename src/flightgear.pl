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

Confirm Python virtual environment containing FlightGear is correctly
configured.

==
?- py_version.
Interactive session; added `.` to Python `sys.path`
Janus 1.5.2 embeds Python 3.13.3 (main, Aug 14 2025, 11:53:40) [GCC 14.2.0]
Janus: using venv from '/home/mike/aircraft_state_estimator/flightgear'
==
*/

:- use_module(library(janus)).
:- use_module(library('plot/plotter')).
:- use_module(library(autowin)).


user:test :-
    process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   ['--httpd=8080', '--airport=NZWN', '--runway=34'],
                   [stderr(pipe(Out))]),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    flightgear_http_connection(HTTP_Conn),
    set_prop('/sim/current-view/view-number', HTTP_Conn, 1),   % "Chase" view. Takes a few seconds to load
    align_heading_indicator,
    set_prop('/controls/engines/engine/throttle', HTTP_Conn, 1.0),
    steer_heading_on_ground.

%    thread_create(fly_heading(340), _, [detached(true)]),
%    thread_create(fly_pitch(4), _, [detached(true)]).


%! steer_heading_on_ground is det.

steer_heading_on_ground :-
    flightgear_http_connection(HTTP_Conn),
    pid_controller(on_ground,                                                                          % Guard
                   heading_on_ground,                                                                  % Setpoint
                   get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn),    % State_Value
                   set_prop('/controls/flight/rudder', HTTP_Conn),                                     % Control
                   direction_difference,                                                               % Error calculation
                   0.03,                                                                               % P
                   0,                                                                                  % I
                   0).                                                                                 % D


%!  on_ground(+HTTP_Conn) is semidet.

on_ground(HTTP_Conn) :-
    get_prop('/position/altitude-agl-ft', HTTP_Conn, Altitude_AGL_Feet),
    Altitude_AGL_Feet < 10.


%!  heading_on_ground(-Heading) is det.

heading_on_ground(338).

%!  fly_heading(+Heading:integer) is det.

fly_heading(Heading) :-
    flightgear_http_connection(HTTP_Conn),
    repeat,
    sleep(0.2),
    get_prop('/position/altitude-agl-ft', HTTP_Conn, Altitude_AGL_Feet),
    Altitude_AGL_Feet >= 10,

    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn, Indicated_Heading_Deg),
    get_prop('/instrumentation/attitude-indicator/indicated-roll-deg', HTTP_Conn, Indicated_Roll_Deg),
    direction_difference(Indicated_Heading_Deg, Heading, Heading_Error_Deg),    % +ve means right of intended track
    (   Heading_Error_Deg < -10
    ->  Required_Roll_Deg = 10

    ;   -10 =< Heading_Error_Deg, Heading_Error_Deg =< +10
    ->  Required_Roll_Deg is -Heading_Error_Deg

    ;   otherwise
    ->  Required_Roll_Deg = -10
    ),
    direction_difference(Indicated_Roll_Deg, Required_Roll_Deg, Roll_Error_Deg),  % +ve means roll left to null
    clamped(-10 * Roll_Error_Deg / 180.0, -0.5, +0.5, Aileron),

    % writeln(fly_heading(heading(Indicated_Heading_Deg), roll_error(Roll_Error_Deg), required_roll(Required_Roll_Deg), actual_roll(Indicated_Roll_Deg), aileron(Aileron))),
    set_prop('/controls/flight/aileron', HTTP_Conn, Aileron),
    fail.



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


%!  fly_indicated_airspeed(+Required_Speed_KT) is det.
%
%   Adjust trim to fly an indicated airspeed
%
%   NOT TESTED

fly_indicated_airspeed(Required_Speed_KT) :-
    flightgear_http_connection(HTTP_Conn),
    set_prop('/controls/flight/elevator-trim', HTTP_Conn, 0),
    Sample_Time = 0.2,
    repeat,
    sleep(Sample_Time),
    get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', HTTP_Conn, Indicated_Speed_KT),
    writeln(ias(Indicated_Speed_KT)),
    Indicated_Speed_KT > 40,
    Airspeed_Error is Indicated_Speed_KT - Required_Speed_KT,   % +ve if speed too high
    get_prop('/controls/flight/elevator-trim', HTTP_Conn, Current_Elevator_Trim),
    New_Elevator_Trim is Current_Elevator_Trim - 0.001 * Airspeed_Error * Sample_Time,
    writeln(fly_indicated_airspeed(ias(Indicated_Speed_KT), elevator_trim(New_Elevator_Trim))),
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


%!  pid_controller(:Guard, :Setpoint_Pred, :State_Value_Pred, :Control_Pred, :Error_Pred, +P, +I, +D) is det.
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

:- meta_predicate
    pid_controller(:, :, :, :, :, +, +, +).

pid_controller(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D) :-
    pid_plot(P),
    call(State_Value_Pred, State_Value),
    sample_time(Sample_Time),
    thread_create(pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Sample_Time, 0, State_Value), _, [detached(true)]).


%! pid_controller_1(:Setpoint_Pred, :State_Value_Pred, :Control_Pred, :Error_Pred, +P, +I, +D, +Sample_Time, +Error_Sum, +Previous_Error) is det.

:- meta_predicate
    pid_controller_1(:, :, :, :, :, +, +, +, +, +, +).

pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Sample_Time, Error_Sum, Previous_Error) :-
    (   Guard
    ->  call(Setpoint_Pred, Setpoint),
        call(State_Value_Pred, State_Value),
        call(Error_Pred, Setpoint, State_Value, Error),
        Error_Sum_1 is Error_Sum + Error,
        Control is P * Error + I * Error_Sum * Sample_Time + D * (Error - Previous_Error) / Sample_Time,
        call(Control_Pred, Control)
    ;   true
    ),
    sleep(Sample_Time),
    pid_controller_1(Guard, Setpoint_Pred, State_Value_Pred, Control_Pred, Error_Pred, P, I, D, Sample_Time, Error_Sum_1, Error).


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


%!  flightgear_http_connection(-HTTPConnection) is det.

flightgear_http_connection(HTTPConnection) :-
    py_call('flightgear_python.fg_if':'HTTPConnection'(localhost, 8080, timeout_s=10), HTTPConnection).


%!  set_prop(+Property_Path, +HTTP_Conn, +Value) is det.

set_prop(Property_Path, HTTP_Conn, Value) :-
    py_call(HTTP_Conn:set_prop(Property_Path, Value)).

%!  get_prop(+Property_Path, +HTTP_Conn, -Value) is det.

get_prop(Property_Path, HTTP_Conn, Value) :-
    py_call(HTTP_Conn:get_prop(Property_Path), Value).


%!  pid_plot(-P) is det.

pid_plot(P) :-
    new(W, auto_sized_picture('PID')),
    send(W, max_size, size(1800, 600)),
    send(W, display, new(P, plotter)),
    send(P, axis, plot_axis(x, 0, 200, @default, 1500)),
    send(P, axis, plot_axis(y, -1, 1, @default, 400)),
    send(P, graph, new(G, plot_graph)),
    send(W, open).

