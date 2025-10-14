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

Initally used the Python interface to FlightGear via Janus but this
proved complex and an issue with an uninitialised stdout stream on the
sub-process used for the socket interface was never resolved.

URL for testing http: /home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage --httpd=8080 --fg-root=/home/mike/.fgfs/fgdata_2024_1
*/

:- use_module(library(socket)).
:- use_module(library('plot/plotter')).
:- use_module(library('plot/axis')).
:- use_module(library(autowin)).
:- use_module(library(lists)).
:- use_module(library(pce)).
:- use_module(library(process)).
:- use_module(library(readutil)).
:- use_module(library(sgml_write)).
:- use_module(library(pairs)).
:- use_module(library(http/http_client)).
:- use_module(library(http/json)).

%!  test is det.

user:test :-
    write_swi_fg_xml_file,  % Supply the specification of the SWI/FG interface to FG
    fg_root(FG_ROOT),
    format(string(FG_ROOT_Arg), '--fg-root=~w', [FG_ROOT]),
    udp_ports(TX_Port, RX_Port),
    http_port(HTTP_Port),
    polling_frequency(Polling_Frequency),
    format(string(Generic_TX_Arg), '--generic=socket,out,~w,localhost,~w,udp,swi_fg', [Polling_Frequency, TX_Port]),
    format(string(Generic_RX_Arg), '--generic=socket,in,~w,localhost,~w,udp,swi_fg', [Polling_Frequency, RX_Port]),
    format(string(HTTP_Arg), '--httpd=~w', [HTTP_Port]),
    process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   [FG_ROOT_Arg, Generic_TX_Arg, Generic_RX_Arg,
                    HTTP_Arg,         % Slow but good for ad hoc gets and sets
                    '--airport=NZWN',
                    '--runway=34'],
                   [stderr(pipe(Out))]),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    thread_create(track_aircraft_instruments(TX_Port), _, [detached(true)]),
    thread_create(send_aircraft_controls(RX_Port), _,  [detached(true)]).

%    thread_create(fly_heading(340), _, [detached(true)]),
%    thread_create(fly_pitch(4), _, [detached(true)]).


%!  track_aircraft_instruments(+Port) is det.
%
%   Maintain a database of the aircraft instruments specified in
%   swi_fg_input/2

:- dynamic
    aircraft_instruments/1.

track_aircraft_instruments(Port) :-
    findall(Node_Name, swi_fg_input(Node_Name, _), Node_Names),
    udp_socket(Socket),
    tcp_bind(Socket, Port),
    repeat,
    udp_receive(Socket, Tuple, _, [as(term)]),
    round_to_square_list(Tuple, Values),
    pairs_keys_values(Pairs, Node_Names, Values),
    dict_pairs(Dict, aircraft_instruments, Pairs),
    with_mutex(swi_fg_instruments,
               (   retractall(aircraft_instruments(_)),
                   assert(aircraft_instruments(Dict)))),
    fail.

%! round_to_square_list(+Tuple, -List) is det.

round_to_square_list((A,B), [A|T]) :-
    !,
    round_to_square_list(B, T).
round_to_square_list(A, [A]).


%!  send_aircraft_controls(+RX_Port) is det.

:- dynamic
    aircraft_controls/1.

send_aircraft_controls(RX_Port) :-
    findall(Node_Name, swi_fg_output(Node_Name, _), Node_Names),
    initial_control_pairs(Node_Names, Pairs),
    dict_pairs(Dict, aircraft_controls, Pairs),
    with_mutex(swi_fg_controls,
               (   retractall(aircraft_controls(_)),
                   assert(aircraft_controls(Dict)))),   % Initialise
    polling_frequency(Polling_Frequency),
    Polling_Delay is 1 / Polling_Frequency,
    udp_socket(Socket),
    repeat,
    with_mutex(swi_fg_controls, aircraft_controls(Dict)),
    control_values(Node_Names, Dict, Node_Values),
    atomic_list_concat(Node_Values, ',', Node_Values_Atom),
    udp_send(Socket, Node_Values_Atom, localhost:RX_Port, [as(string)]),
    sleep(Polling_Delay),
    fail.


%!  control_values(+Node_Names, +Dict, -Values) is det.

control_values([], _, []).
control_values([Node_Name|T1], Dict, [Dict.Node_Name|T2]) :-
    control_values(T1, Dict, T2).


%! initial_control_pairs(+Node_Names, -Pairs) is det.

initial_control_pairs([], []).
initial_control_pairs([Node_Name|T1], [Node_Name-0|T2]) :-
    initial_control_pairs(T1, T2).


%! udp_ports(-TX_Port, -RX_Port) is det.
%
%   Port directions are from the point of view of FlightGear

udp_ports(5501, 5502).


%!  http_port(-Port) is det.

http_port(8080).


%!  fg_root(-Dir) is det.
%
%  Location of FlightGear main data directory

fg_root('/home/mike/.fgfs/fgdata_2024_1').


%!  polling_frequency(-Frequency) is det.
%
%   @arg Frequency Polling frequency in Hz

polling_frequency(10).


%!  write_swi_fg_xml_file is det.
%
%   Write the XML file that specifies the data to be read and written by
%   the FlightGear "generic" (UDP) interface

write_swi_fg_xml_file :-
    fg_root(FG_Root),
    format(string(Dir), '~w/Protocol', [FG_Root]),
    directory_file_path(Dir, 'swi_fg.xml', Path),
    setup_call_cleanup(open(Path, write, Out),
                       swi_fg_xml_write_1(Out),
                       close(Out)).


%!  swi_fg_xml_write_1(+Out:stream) is det.

swi_fg_xml_write_1(Out) :-
    findall(Chunk, chunk(input, Chunk), Output_Chunks),   % Inputs to FG are outputs from SWI
    findall(Chunk, chunk(output, Chunk), Input_Chunks),
    DOM = [element('PropertyList',
                   [],
                   [element(generic,
                            [],
                            [element(output,
                                     [],
                                     [element(line_separator,[],[newline]),
                                      element(var_separator,[],[',']),
                                      element(binary_mode,[], [false])|Output_Chunks]),
                            element(input,
                                     [],
                                     [element(line_separator,[],[newline]),
                                      element(var_separator,[],[',']),
                                      element(binary_mode,[], [false])|Input_Chunks])])])],
    xml_write(Out, DOM, []).


%!  chunk(+Direction, -Chunk) is nondet.

chunk(Direction, Chunk) :-
    (   Direction == input
    ->  swi_fg_input(Node_Name, Format)

    ;   Direction == output
    ->  swi_fg_output(Node_Name, Format)
    ),
    Chunk = element(chunk,
                    [],
                    [element(name, [], [Node_Name]),
                     element(format, [], [Format]),
                     element(node, [], [Node_Name])]).


%!  swi_fg_input(-Node_ID:atom, -Format:atom).

swi_fg_input('instrumentation/heading-indicator/indicated-heading-deg', '%d').
swi_fg_input('position/altitude-agl-ft', '%d').
swi_fg_input('instrumentation/airspeed-indicator/indicated-speed-kt', '%d').
swi_fg_input('instrumentation/attitude-indicator/indicated-pitch-deg', '%d').


%!  swi_fg_output(-Node_ID:atom, -Format:atom).

swi_fg_output('controls/flight/rudder', '%d').


%! steer_heading_on_ground(+Plotter) is det.

steer_heading_on_ground(Plotter) :-
    P = 0.03,
    I = 0.03,
    D = 0.0,
    Control_Min = -0.5,
    Control_Max = +0.5,
    pid_controller(on_ground,                                                               % Guard
                   required_heading,                                                                   % Setpoint
                   http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn),    % State_Value
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
    http_get_prop('/position/altitude-agl-ft', HTTP_Conn, Altitude_AGL_Feet),
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
                   http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', HTTP_Conn),    % State_Value
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
    http_get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', HTTP_Conn, Indicated_Speed_KT),
    http_get_prop('/instrumentation/attitude-indicator/indicated-pitch-deg', HTTP_Conn, Indicated_Pitch_Deg),
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
    http_get_prop('/instrumentation/magnetic-compass/indicated-heading-deg', Magnetic_Compass_Indicated_Heading),
    http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Unaligned_Indicated_Heading),
    http_get_prop('/instrumentation/heading-indicator/align-deg', Initial_Align_Deg),
    direction_difference(Magnetic_Compass_Indicated_Heading, Unaligned_Indicated_Heading, Heading_Indicator_Alignment_Error),
    Align_Deg is integer(Initial_Align_Deg + Heading_Indicator_Alignment_Error) mod 360,
    http_set_prop('/instrumentation/heading-indicator/align-deg', Align_Deg).


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


%!  http_get_prop(+Property_Path, -Value) is det.

http_get_prop(Property_Path, Value) :-
    http_port(Port),
    format(atom(Path), '/json/~w', [Property_Path]),
    http_get([protocol(http), host(localhost), port(Port), path(Path)], json(JSON), []),
    memberchk(value=Value, JSON).


%!  http_set_prop(+Property_Path, +Value) is det.

http_set_prop(Property_Path, Value) :-
    http_port(Port),
    format(atom(Path), '/json/~w', [Property_Path]),
    URL = [protocol(http), host(localhost), port(Port), path(Path)],
    http_get(URL, json(JSON), []),
    memberchk(type=Type, JSON),
    http_post(URL, json(#{path:Property_Path, type:Type, value:Value}), X, []).


%!  pid_plotter(-P) is det.

pid_plotter(P) :-
    new(W, auto_sized_picture('PID')),
    send(W, max_size, size(1800, 600)),
    send(W, display, new(P, plotter)),
    send(P, axis, plot_axis(x, 0, 200, @(default), 1500)),
    send(P, axis, plot_axis(y, -1, 1, @(default), 400)),
    send(W, open).






