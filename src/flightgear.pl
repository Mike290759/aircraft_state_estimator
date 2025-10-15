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

URL for testing http:
/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage --httpd=8080 --fg-root=/home/mike/.fgfs/fgdata_2024_1 --airport=NZWN --runway=34
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
:- use_module(library(http/http_server)).


%!  airport_and_runway(-Airport, -Runway) is det.

airport_and_runway('NZWN', 34).


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
    airport_and_runway(Airport, Runway),
    format(string(Airport_Arg), '--airport=~w', [Airport]),
    format(string(Runway_Arg), '--runway=~w', [Runway]),
    process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   [FG_ROOT_Arg, Generic_TX_Arg, Generic_RX_Arg,
                    HTTP_Arg,         % Slow but good for ad hoc gets and sets
                    Airport_Arg,
                    Runway_Arg,
                   '--disable-ai-traffic'],
                   [stderr(pipe(Out))]),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    http_set_prop('/sim/current-view/view-number', 1),
    pid_plotter(Plotter),
    align_heading_indicator,
    set_brakes(1.0),
    http_set_prop('/controls/engines/engine/throttle', 1.0),
    thread_create(get_udp_properties(TX_Port), _, [detached(true)]),
    thread_create(send_udp_properties(RX_Port), _,  [detached(true)]),
    repeat,
    (   udp_input_properties(_),
        udp_input_properties(_)
    ->  !,
        cycle_rudder,   % Look at the GUI to confirm that the ruidder moves
        set_brakes(0.05),
        steer_heading_on_ground(Plotter)
    ;   sleep(0.1),
        fail
    ).


%    thread_create(fly_heading(340), _, [detached(true)]),
%    thread_create(fly_pitch(4), _, [detached(true)]).


%!  get_udp_properties(+Port) is det.
%
%   Maintain a database of the properties specified in
%   swi_fg_input/3 in the fact udp_input_properties/1

:- dynamic
    udp_input_properties/1.

get_udp_properties(Port) :-
    findall(Node_Name, swi_fg_input(Node_Name, _, _), Node_Names),
    udp_socket(Socket),
    tcp_bind(Socket, Port),
    repeat,
    udp_receive(Socket, Tuple, _, [as(term)]),
    round_to_square_list(Tuple, Values),
    pairs_keys_values(Pairs, Node_Names, Values),
    dict_pairs(Dict, #, Pairs),
    with_mutex(swi_fg_input_properties,
               (   retractall(udp_input_properties(_)),
                   assert(udp_input_properties(Dict)))),
    fail.

%! round_to_square_list(+Tuple, -List) is det.

round_to_square_list((A,B), [A|T]) :-
    !,
    round_to_square_list(B, T).
round_to_square_list(A, [A]).


%!  send_udp_properties(+Port) is det.

:- dynamic
    udp_output_properties/1.

send_udp_properties(Port) :-
    findall(Node_Name, swi_fg_output(Node_Name, _, _), Node_Names),
    initial_output_pairs(Node_Names, Pairs),
    dict_pairs(Initial_Dict, #, Pairs),
    with_mutex(swi_fg_output_properties,
               (   retractall(udp_output_properties(_)),
                   assert(udp_output_properties(Initial_Dict)))),   % Initialise
    polling_frequency(Polling_Frequency),
    Polling_Delay is 1 / Polling_Frequency,
    udp_socket(Socket),
    repeat,
    with_mutex(swi_fg_output_properties, udp_output_properties(Dict)),
    output_values(Node_Names, Dict, Node_Values),
    atomic_list_concat(Node_Values, ',', Node_Values_Atom),
    format(atom(Tuple_String), '~w~n', [Node_Values_Atom]),
    udp_send(Socket, Tuple_String, localhost:Port, []),
    sleep(Polling_Delay),
    fail.


%!  output_values(+Node_Names, +Dict, -Values) is det.

output_values([], _, []).
output_values([Node_Name|T1], Dict, [Dict.Node_Name|T2]) :-
    output_values(T1, Dict, T2).


%! initial_output_pairs(+Node_Names, -Pairs) is det.

initial_output_pairs([], []).
initial_output_pairs([Node_Name|T1], [Node_Name-Value|T2]) :-
    swi_fg_output(Node_Name, Type, _),
    (   Type == float
    ->  Value = 0.0
    ;   Value = 0
    ),
    initial_output_pairs(T1, T2).


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

polling_frequency(5).


%!  swi_fg_input(-Node_ID:atom, -Type, -Format:atom).

swi_fg_input('/instrumentation/heading-indicator/indicated-heading-deg', int, '%d').
swi_fg_input('/position/altitude-agl-ft', int, '%d').
swi_fg_input('/instrumentation/airspeed-indicator/indicated-speed-kt', int, '%d').
swi_fg_input('/instrumentation/attitude-indicator/indicated-pitch-deg', int, '%d').


%!  swi_fg_output(-Node_ID:atom, -Type, -Format:atom).

swi_fg_output('/controls/flight/rudder', float, '%f').
swi_fg_output('/controls/flight/elevator-trim', float, '%f').


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
    ->  swi_fg_input(Node_Name, Type, Format)

    ;   Direction == output
    ->  swi_fg_output(Node_Name, Type, Format)
    ),
    Chunk = element(chunk,
                    [],
                    [element(name, [], [Node_Name]),
                     element(type, [], [Type]),
                     element(format, [], [Format]),
                     element(node, [], [Node_Name])]).



%! steer_heading_on_ground(+Plotter) is det.

steer_heading_on_ground(Plotter) :-
    pid_controller(on_ground,                                                                 % Guard
                   required_heading,                                                          % Setpoint
                   udp_get_prop('/instrumentation/heading-indicator/indicated-heading-deg'),  % State_Value
                   udp_set_prop('/controls/flight/rudder'),                                   % Control
                   direction_difference,                                                      % Error calculation
                   0.04,                                                                      % P
                   0.0,                                                                      % I
                   0.0,                                                                       % D
                   -1.0,                                                                      % Control_Min
                   +1.0,                                                                      % Control_Max
                   Plotter,
                   0.01,                                                                      % Plotting Y-scale factor
                   green).

%!  set_brakes(+Value) is det.

set_brakes(Value) :-
    http_set_prop('/controls/gear/brake-left', Value),
    http_set_prop('/controls/gear/brake-right', Value).


%!  cycle_rudder is det.

cycle_rudder :-
    between(1, 2, _),
    member(Rudder, [-0.8, 0.0, 0.8, 0.0]),
    udp_set_prop('/controls/flight/rudder', Rudder),
    sleep(1.0),
    fail.
cycle_rudder.


%!  on_ground is semidet.

on_ground :-
    udp_get_prop('/position/altitude-agl-ft', Altitude_AGL_Feet),
    Altitude_AGL_Feet < 30.  % Need to set the QNH!


%!  required_heading(-Heading) is det.

required_heading(Heading) :-
    airport_and_runway(_, Runway),
    Heading is 10 * Runway.


%!  fly_heading is det.

fly_heading(Plotter) :-
    P = 0.01,
    I = 0.0,
    D = 0.0,
    Control_Min = -0.5,
    Control_Max = +0.5,
    pid_controller(\+ on_ground,                                                                % Guard
                   required_heading,                                                            % Setpoint
                   udp_get_prop('/instrumentation/heading-indicator/indicated-heading-deg'),    % State_Value
                   udp_set_prop('/controls/flight/aileron'),                                    % Control
                   direction_difference,                                                        % Error calculation
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
    Sample_Time = 0.2,
    repeat,
    sleep(Sample_Time),
    udp_get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', Indicated_Speed_KT),
    udp_get_prop('/instrumentation/attitude-indicator/indicated-pitch-deg', Indicated_Pitch_Deg),
    (   Indicated_Speed_KT < 50
    ->  Required_Pitch_Deg_1 = 4
    ;   Required_Pitch_Deg_1 = Required_Pitch_Deg
    ),
    Pitch_Error is Indicated_Pitch_Deg - Required_Pitch_Deg_1,   % +ve if pitch too high
    clamped(0.3 * Pitch_Error * Sample_Time, -0.5, +0.5, New_Elevator_Trim),
    udp_set_prop('/controls/flight/elevator-trim', New_Elevator_Trim),
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
    http_set_prop('/instrumentation/heading-indicator/align-deg', Align_Deg),
    http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Heading_Indicator_Heading),
    format('Magnetic heading=~w, Heading Indicator heading=~w~n', [Magnetic_Compass_Indicated_Heading, Heading_Indicator_Heading]).


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
    pid_controller(0, 1, 1, 1, 3, +, +, +, +, +, +, +, +).

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
    pid_controller_1(0, 1, 1, 1, 3, +, +, +, +, +, +, +, +, +, +, +).

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

sample_time(0.3).



%!  direction_difference(+H1, +H2, -Diff) is det.
%
%   Diff = H1 - H2
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


%!  udp_get_prop(+Property_Path, -Value) is det.

udp_get_prop(Property_Path, Value) :-
    with_mutex(swi_fg_input_properties, udp_input_properties(DICT)),
    Value = DICT.Property_Path.


%!  udp_set_prop(+Property_Path, +Value) is det.

udp_set_prop(Property_Path, Value) :-
    with_mutex(swi_fg_output_properties,
               (   retract(udp_output_properties(DICT_IN)),
                   get_dict(Property_Path, DICT_IN, _, DICT_OUT, Value),
                   assert(udp_output_properties(DICT_OUT)))).


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
    http_post(URL, json(#{path:Property_Path, type:Type, value:Value}), _, []).


%!  pid_plotter(-P) is det.

pid_plotter(P) :-
    new(W, auto_sized_picture('PID')),
    send(W, max_size, size(1800, 600)),
    send(W, display, new(P, plotter)),
    send(P, axis, plot_axis(x, 0, 200, @(default), 1500)),
    send(P, axis, plot_axis(y, -1, 1, @(default), 400)),
    send(W, open).






