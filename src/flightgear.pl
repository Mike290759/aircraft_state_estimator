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

:- use_module(library('plot/axis')).
:- use_module(library('plot/plotter')).
:- use_module(library(autowin)).
:- use_module(library(debug)).
:- use_module(library(error)).
:- use_module(library(filesex)).
:- use_module(library(http/http_client)).
:- use_module(library(http/http_server)).
:- use_module(library(lists)).
:- use_module(library(pairs)).
:- use_module(library(pce)).
:- use_module(library(process)).
:- use_module(library(readutil)).
:- use_module(library(sgml_write)).
:- use_module(library(socket)).

:- debug(swifg).

:- meta_predicate
    pid_controller(+, 0, 1, 1, 2, 1, +, +, +, +, +, +).

:- meta_predicate
    pid_controller_1(+, 0, 1, 1, 2, 1, +, +, +, +, +, +, +, +, +, +, +, +).

%!  airport_and_runway(-Airport, -Runway, -Heading) is det.

airport_and_runway('NZWN', 34, 338).


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
    airport_and_runway(Airport, Runway, _),
    format(string(Airport_Arg), '--airport=~w', [Airport]),
    format(string(Runway_Arg), '--runway=~w', [Runway]),
    process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   [FG_ROOT_Arg, Generic_TX_Arg, Generic_RX_Arg,
                    HTTP_Arg,         % Slow but good for ad hoc gets and sets
                    Airport_Arg,
                    Runway_Arg,
                   '--disable-ai-traffic'],
                   [stderr(pipe(Out))]),
    debug(swifg, 'Waiting for FlightGear to start', []),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    debug(swifg, 'FlightGear started', []),
    set_brakes(1.0),
    align_heading_indicator,
    http_get_prop('/instrumentation/attitude-indicator/indicated-pitch-deg', Indicated_Pitch_Deg_On_Ground),
    set_qfe,
    start_flightgear_udp_interface,  % Blocks until first tuple received from FlightGear
    cycle_rudder,                    % Look at the GUI to confirm that the rudder moves
    http_set_prop('/controls/engines/engine/throttle', 1.0),
    http_set_prop('/sim/current-view/view-number', 1),
    set_brakes(0.0),
    steer_heading_on_ground,
    fly_heading,
    climb(200, 3000, Indicated_Pitch_Deg_On_Ground).


%!  set_brakes(+Value) is det.

set_brakes(Value) :-
    http_set_prop('/controls/gear/brake-left', Value),
    http_set_prop('/controls/gear/brake-right', Value).


%!   set_qfe is det.
%
%   Set altimeter to adjust indicated altitude to current altitude

set_qfe :-
     http_get_prop('/position/altitude-agl-ft', Actual_Altitude),
     http_get_prop('instrumentation/altimeter/indicated-altitude-ft', Current_Indicated_Altitude),
     http_get_prop('instrumentation/altimeter/setting-hpa', Current_HPA),
     QFE is Current_HPA + (Actual_Altitude - Current_Indicated_Altitude) / 33.7,
     http_set_prop('instrumentation/altimeter/setting-hpa', QFE).


%!  cycle_rudder is det.

cycle_rudder :-
    member(Rudder, [-0.8, 0.0, 0.8, 0.0]),
    udp_set_prop('/controls/flight/rudder', Rudder),
    sleep(1.0),
    fail.
cycle_rudder.


%!  on_ground is semidet.

on_ground :-
    udp_get_prop('/position/altitude-agl-ft', Altitude_AGL_Feet),
    Altitude_AGL_Feet < 30.


%!  required_heading(-Heading) is det.

required_heading(Heading) :-
    airport_and_runway(_, _, Heading).



%! steer_heading_on_ground is det.

steer_heading_on_ground :-
    pid_controller(ground_steering,                                                           % PID_Id
                   on_ground,                                                                 % Guard
                   required_heading,                                                          % Setpoint
                   udp_get_prop('/instrumentation/heading-indicator/indicated-heading-deg'),  % State_Value
                   most_recent_error,                                                         % Error conditioning
                   udp_set_prop('/controls/flight/rudder'),                                   % Control
                   0.03,                                                                      % P
                   0.0,                                                                       % I
                   0.0,                                                                       % D
                   -1.0,                                                                      % Control_Min
                   +1.0,                                                                      % Control_Max
                   200).                                                                      % Y_Scale_Max


%!  fly_heading is det.

fly_heading :-
    pid_controller(fly_heading,                                                               % PID_Id
                   \+ on_ground,                                                              % Guard
                   required_roll,                                                             % Setpoint
                   udp_get_prop('/instrumentation/attitude-indicator/indicated-roll-deg'),    % State_Value
                   most_recent_error,                                                         % Error conditioning
                   udp_set_prop('/controls/flight/aileron'),                                  % Control
                   0.01,                                                                      % P
                   0.001,                                                                     % I
                   0.0,                                                                       % D
                   -1.0,                                                                      % Control_Min
                   +1.0,                                                                      % Control_Max
                   200).                                                                      % Y_Scale_Max



%!  climb(+Vertical_Rate_FPM, +Target_Altitude_FT, +Indicated_Pitch_Deg_On_Ground) is det.
%
%   Adjust trim to fly at the specified Vertical_Rate_FPM until reaching Target_Altitude_FT
%
%   P = -0.0001, I = 0,D = 0 stable but did not completely null the error
%   P = 0, I = -0.000001, D = 0 damped oscillation until error nulled at 300 seconds

climb(Vertical_Rate_FPM, Target_Altitude_FT, Indicated_Pitch_Deg_On_Ground) :-
    pid_controller(climb,                                                                     % PID_Id
                   true,                                                                      % Guard
                   required_vertical_rate(Vertical_Rate_FPM, Target_Altitude_FT),             % Setpoint
                   vertical_rate_estimate(Indicated_Pitch_Deg_On_Ground),                     % State_Value
                   mean_of_last_n_seconds(0.5),                                               % Error conditioning
                   udp_set_prop('/controls/flight/elevator-trim'),                            % Control
                   -0.0001,                                                                   % P
                   -0.00005,                                                                  % I
                   0.0,                                                                       % D
                   -1.0,                                                                      % Control_Min
                   +1.0,                                                                      % Control_Max
                   2000).                                                                     % Y_Scale_Max

%!  required_roll(-Required_Roll) is det.

required_roll(Required_Roll) :-
    udp_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Indicated_Heading),
    required_heading(Required_Heading),
    angular_difference(Required_Heading, Indicated_Heading, Direction_Error),
    (   Direction_Error =< -10
    ->  Required_Roll = -10.0
    ;   Direction_Error >= 10
    ->  Required_Roll = +10.0
    ;   Required_Roll = Direction_Error
    ).


%! required_vertical_rate(+Target_Vertical_Rate_FPM,
%!                        +Target_Altitude_FT,
%!                        -Required_Vertical_Rate_FPM) is det.

required_vertical_rate(Target_Vertical_Rate_FPM, Target_Altitude_FT, Required_Vertical_Rate_FPM) :-
    udp_get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', Indicated_Speed_KT),
    udp_get_prop('/instrumentation/altimeter/indicated_altitude-ft', Indicated_Altitude_FT),
    (   Target_Vertical_Rate_FPM > 0,
        Indicated_Altitude_FT >= Target_Altitude_FT
    ->  Required_Vertical_Rate_FPM = 0.0

    ;   Target_Vertical_Rate_FPM < 0,
        Indicated_Altitude_FT =< Target_Altitude_FT
    ->  Required_Vertical_Rate_FPM = 0.0

    ;   Indicated_Speed_KT < 55
    ->  Required_Vertical_Rate_FPM = 0.0

    ;   Required_Vertical_Rate_FPM = Target_Vertical_Rate_FPM
    ).


%!  vertical_rate_estimate(Indicated_Pitch_Deg_On_Ground, -Vertical_Rate_FPM) is det.
%
%   Rate_of_Climb = Airspeed x sin(Gamma)
%   where
%   Gamma = Attitude - Angle_of_Incidence
%
%   For C172 Angle_of_Incidence = 1.5 degrees
%
%   udp_get_prop('/instrumentation/vertical-speed-indicator/indicated-speed-fpm')

:- debug(swifg(climb)).

vertical_rate_estimate(Indicated_Pitch_Deg_On_Ground, Vertical_Rate_FPM) :-
    udp_get_prop('/instrumentation/airspeed-indicator/indicated-speed-kt', Indicated_Speed_KT),
    udp_get_prop('/instrumentation/attitude-indicator/indicated-pitch-deg', Indicated_Pitch_Deg),
    Gamma is Indicated_Pitch_Deg - Indicated_Pitch_Deg_On_Ground + 1.5,   % 1.5 is wing angle of incidence
    (   Indicated_Speed_KT > 45
    ->  Vertical_Rate_FPM is 101.3 * Indicated_Speed_KT * sin(pi * Gamma / 180.0)
    ;   Vertical_Rate_FPM = 0
    ),
    debug(swifg(climb), 'Indicated_Pitch_Deg=~2f, Gamma=~2f, Indicated_Speed_KT=~0f, Vertical_Rate_FPM=~0f', [Indicated_Pitch_Deg, Gamma, Indicated_Speed_KT, Vertical_Rate_FPM]).


%!  align_heading_indicator is det.
%
%   Align the DI with the magnetic compass

align_heading_indicator :-
    http_get_prop('/instrumentation/magnetic-compass/indicated-heading-deg', Magnetic_Compass_Indicated_Heading),
    http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Unaligned_Indicated_Heading),
    http_get_prop('/instrumentation/heading-indicator/align-deg', Initial_Align_Deg),
    angular_difference(Magnetic_Compass_Indicated_Heading, Unaligned_Indicated_Heading, Heading_Indicator_Alignment_Error),
    Align_Deg is integer(Initial_Align_Deg + Heading_Indicator_Alignment_Error) mod 360,
    http_set_prop('/instrumentation/heading-indicator/align-deg', Align_Deg),
    http_get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Heading_Indicator_Heading),
    debug(swifg, 'Magnetic heading=~w, Heading Indicator heading=~w', [Magnetic_Compass_Indicated_Heading, Heading_Indicator_Heading]).



%!  pid_controller(+PID_Id, :Guard, :Setpoint_Pred, :State_Value_Pred,
%!                 :Conditioned_Error_Pred, :Control_Pred, +P, +I, +D,
%!                 +Control_Min, +Control_Max,
%!                 +Y_Scale_Max) is det.
%
%   PID controller to reduce the mismatch between the input setpoint
%   and the desired state value. This predicate runs forever on its own
%   thread.
%
%   @arg PID_Id identifier for the PID controller
%   @arg Guard run PID loop iff Guard succeeds
%   @arg Setpoint_Pred name of goal to get the desired set point.
%   e.g if Setpoint_Pred was required_roll_angle,
%   required_roll_angle(-X) will be called.
%
%   @arg State_Value_Pred name of goal to get the current state value e.g if
%   State_Value was roll_angle, roll_angle(X) will be called.
%
%   @arg Conditioned_Error_Pred name of goal to calculate the error from
%   the Setpoint and the State_Value e.g. if Error_Pred was
%   smoothed_error, smoothed_error(Error_History, Smoothed_Error)
%   would be called
%
%   @arg Control_Pred name of goal to apply the output of the PID
%   controller e.g. if Control_Output was set_aileron, set_aileron(X)
%   will be called.
%
%   @arg P float the proportional gain
%   @arg I float the integration gain
%   @arg D float the derivative gain
%   @arg Control_Min float
%   @arg Control_Max float
%   @arg Y_Scale_Max maximum Y value (+ and -)

pid_controller(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Y_Scale_Max) :-
    sample_time(Sample_Time),
    graph(PID_Id, Y_Scale_Max, Error_Plot, Control_Plot, Step_Response_Plot),
    thread_create(pid_controller_1(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, [], 0),
                  _,
                  [detached(true)]).


%!  graph(+PID_Id, +Y_Scale_Max, -Error_Plot, -Control_Plot, -Step_Response_Plot) is det.

graph(PID_Id, Y_Scale_Max, Error_Plot, Control_Plot, Step_Response_Plot) :-
    new(W, auto_sized_picture(PID_Id)),
    send(W, max_size, size(2000, 600)),
    send(W, display, new(Plotter, plotter)),
    send(Plotter, axis, plot_axis(x, 0, 200, @(default), 1500)),
    send(Plotter, axis, plot_axis(y, -Y_Scale_Max, Y_Scale_Max, @(default), 400)),
    send(Plotter, graph, new(Error_Plot, plot_graph)),
    send(Error_Plot, colour, red),
    send(Plotter, graph, new(Control_Plot, plot_graph)),
    send(Control_Plot, colour, blue),
    send(Plotter, graph, new(Step_Response_Plot, plot_graph)),
    send(Step_Response_Plot, colour, green),
    send(W, open).


%! pid_controller_1(+PID_Id, :Setpoint_Pred, :State_Value_Pred,
%!                  :Control_Pred, :Conditioned_Error_Pred, +P, +I, +D,
%!                  +Control_Min, +Control_Max, +Plot_Graph,
%!                  +Y_Scale_Max, +Sample_Time, +Error_History,
%!                  +Elapsed_Time) is det.
%
%   @arg Error_History list of error values, most recent first. Maximum
%   length set by error_history_window_size/1

:- dynamic
    user:capture_step_response/2.

pid_controller_1(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History_0, Elapsed_Time) :-
    setup_call_cleanup(open('step_response.dat', write, Step_Response_Stream),
                       pid_controller_2(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History_0, Elapsed_Time, Step_Response_Stream),
                       close(Step_Response_Stream)).

pid_controller_2(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History_0, Elapsed_Time, Step_Response_Stream) :-
    Guard,
    !,
    (   user:capture_step_response(PID_Id, Elapsed_Time, Control)
    ->  call(Control_Pred, Control),
        call(State_Value_Pred, State_Value),
        in_pce_thread(send(Step_Response_Plot, append, Elapsed_Time, State_Value)),
        format(Step_Response_Stream, '~q.~n', [p(Elapsed_Time, State_Value)])

    ;   call(Setpoint_Pred, Setpoint),
        call(State_Value_Pred, State_Value),
        difference(Setpoint, State_Value, Error),
        error_history(Error, Error_History_0, Error_History),
        call(Conditioned_Error_Pred, Error_History, PID_Input_Error),
        in_pce_thread(send(Error_Plot, append, Elapsed_Time, PID_Input_Error)),
        sum_list(Error_History, Integral),
        derivative(Error_History, Derivative),
        clamped(P * PID_Input_Error + I * Integral * Sample_Time + D * Derivative, Control_Min, Control_Max, Control),
        call(Control_Pred, Control)
    ),
    Control_Plot_Y is Control * Y_Scale_Max,
    in_pce_thread(send(Control_Plot, append, Elapsed_Time, Control_Plot_Y)),
    Elapsed_Time_1 is Elapsed_Time + Sample_Time,
    sleep(Sample_Time),
    pid_controller_2(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History, Elapsed_Time_1, Step_Response_Stream).

pid_controller_2(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History, Elapsed_Time, Step_Response_Stream) :-
    Elapsed_Time_1 is Elapsed_Time + Sample_Time,
    sleep(Sample_Time),
    pid_controller_2(PID_Id, Guard, Setpoint_Pred, State_Value_Pred, Conditioned_Error_Pred, Control_Pred, P, I, D, Control_Min, Control_Max, Error_Plot, Control_Plot, Step_Response_Plot, Y_Scale_Max, Sample_Time, Error_History, Elapsed_Time_1, Step_Response_Stream).


%!  sample_time(-Sample_Time) is det.
%
%   @arg Sample_Time float sample time in seconds

sample_time(0.2).


%!  error_history(+State_Value, +Error_History, -New_Error_History) is det.

error_history(V, [], L) :-
    !,
    error_history_window_size(N),
    initial_error_history(N, V, L).

error_history(V, L0, L) :-
    append(Front, [_], L0),
    !,
    L = [V|Front].


%!  difference(+A, +B, -C) is det.
%
%   C = A - B
%
%   @arg A number ; degrees/1
%   @arg B number ; degrees/1
%   @arg C number

difference(degrees(A), degrees(B), C) :-
    !,
    angular_difference(A, B, C).
difference(A, B, C) :-
    C is A - B.


%!  angular_difference(+H1, +H2, -Diff) is det.
%
%   Diff = H1 - H2
%
%   @arg Diff [-180, +180]

angular_difference(H1, H2, Diff) :-
    Diff is integer((H1 - H2 + 540)) mod 360 - 180.


%! initial_error_history(+Length:integer, +State_Value, -Error_History:list) is det.

initial_error_history(0, _, []) :-
    !.
initial_error_history(N, V, [V|T]) :-
    NN is N-1,
    initial_error_history(NN, V, T).


%!  error_history_window_size(-Size)
%
%   Size is the number of error values to keep.
%
%   @arg Size integer

error_history_window_size(50).


%!   mean_of_last_n_seconds(N:integer, +Values, -Mean) is det.
%
%   @arg Values list error_history_window_size/1 long, sample_time/1
%   apart

mean_of_last_n_seconds(N, Values, Mean) :-
    sample_time(S),
    Samples_Count is integer(N / S),
    length(L, Samples_Count),
    append(L, _, Values),
    sum_list(L, Sum),
    Mean is Sum / Samples_Count.


%!   derivative(+Error_History, -Derivative) is det.
%
%   @tbd

derivative(_, 0).


%!  most_recent_error(+Error_History, -Error) is det.

most_recent_error([V|_], V).


%!   clamped(+Expr, +Left, +Right, -Clamped) is det.

clamped(Expr, Left, Right, Clamped) :-
    X is Expr,
    (   X =< Left
    ->  Clamped is Left
    ;   X >= Right
    ->  Clamped is Right
    ;   Clamped = X
    ).


%!  swi_fg_input(-Node_ID:atom, -Type, -Format:atom).

swi_fg_input('/instrumentation/airspeed-indicator/indicated-speed-kt', float, '%f').
swi_fg_input('/instrumentation/altimeter/indicated_altitude-ft', float, '%f').
swi_fg_input('/instrumentation/attitude-indicator/indicated-pitch-deg', float, '%f').
swi_fg_input('/instrumentation/attitude-indicator/indicated-roll-deg', float, '%f').
swi_fg_input('/instrumentation/heading-indicator/indicated-heading-deg', float, '%f').
swi_fg_input('/instrumentation/vertical-speed-indicator/indicated-speed-fpm', float, '%f').
swi_fg_input('/position/altitude-agl-ft', float, '%f').


%!  swi_fg_output(-Node_ID:atom, -Type, -Format:atom).

swi_fg_output('/controls/flight/aileron', float, '%f').
swi_fg_output('/controls/flight/elevator-trim', float, '%f').
swi_fg_output('/controls/flight/rudder', float, '%f').


%!  start_flightgea_udp__interface is det.

start_flightgear_udp_interface :-
    initialise_output_property_facts,
    udp_ports(TX_Port, RX_Port),
    thread_create(send_udp_properties(RX_Port), _,  [detached(true)]),
    thread_create(listen_for_udp_properties(TX_Port), _, [detached(true)]),
    repeat,
    flag(udp_rx_message_count, N, N),
    (   N > 0    % Wait for first record from FlightGear
    ->  !
    ;   sleep(0.2),
        fail
    ).


%!  listen_for_udp_properties(+Port) is det.
%
%   Maintain a database of the properties specified in
%   swi_fg_input/3. Each property is stored in its own fact named
%   after the property path.

listen_for_udp_properties(Port) :-
    findall(Property_Path, swi_fg_input(Property_Path, _, _), Property_Paths),
    udp_socket(Socket),
    tcp_bind(Socket, Port),
    repeat,
    udp_receive(Socket, Tuple, _, [as(term)]),
    round_to_square_list(Tuple, Values),
    pairs_keys_values(Pairs, Property_Paths, Values),
    forall(member(Property_Path-Value, Pairs),
           with_mutex(Property_Path,
                      (   functor(Fact, Property_Path, 1),
                          retractall(Fact),
                          arg(1, Fact, Value),
                          assert(Fact)))),
    flag(udp_rx_message_count, N, N+1),
    fail.


%! round_to_square_list(+Tuple, -List) is det.

round_to_square_list((A,B), [A|T]) :-
    !,
    round_to_square_list(B, T).
round_to_square_list(A, [A]).


%!  send_udp_properties(+Port) is det.
%
%   Send properties to FlightGear

send_udp_properties(Port) :-
    polling_frequency(Polling_Frequency),
    Polling_Delay is 1 / Polling_Frequency,
    udp_socket(Socket),
    repeat,
    findall(Value, udp_output_value(Value), Values),
    atomic_list_concat(Values, ',', Values_Atom),
    format(string(UDP_String), '~w~n', [Values_Atom]),
    udp_send(Socket, UDP_String, localhost:Port, []),
    sleep(Polling_Delay),
    fail.


%!  udp_output_value(-Value) is nondet.

udp_output_value(Value) :-
    swi_fg_output(Property_Path, _, _),
    functor(Fact, Property_Path, 1),
    arg(1, Fact, Value),
    call(Fact).


%! initialise_output_property_facts is det.
%
%   Note that the _input_ property facts do not
%   need to be initialised because we wait until
%   that data has arrived from FlightGear before
%   attempting to access those facts.

initialise_output_property_facts :-
    swi_fg_output(Property_Path, Type, _),
    (   Type == float
    ->  Value = 0.0
    ;   Value = 0
    ),
    functor(Fact, Property_Path, 1),
    retractall(Fact),
    arg(1, Fact, Value),
    assert(Fact),
    fail.
initialise_output_property_facts.


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
    ->  swi_fg_input(Property_Path, Type, Format)

    ;   Direction == output
    ->  swi_fg_output(Property_Path, Type, Format)
    ),
    Chunk = element(chunk,
                    [],
                    [element(name, [], [Property_Path]),
                     element(type, [], [Type]),
                     element(format, [], [Format]),
                     element(node, [], [Property_Path])]).



%!  udp_get_prop(+Property_Path, -Value) is det.
%
%   Get the value of a property received from FlightGear

udp_get_prop(Property_Path, Value) :-
    with_mutex(Property_Path,
               udp_get_prop_1(Property_Path, Value)).


%!  udp_get_prop_1(+Property_Path, -Value) is det.

udp_get_prop_1(Property_Path, Value) :-
    functor(Fact, Property_Path, 1),
    arg(1, Fact, V),
    (   Fact
    ->  Value = V
    ;   existence_error(Property_Path, udp_input_property)
    ).


%!  udp_set_prop(+Property_Path:atom, +Value) is det.
%
%   Set the value of property for transmission to FlightGear

udp_set_prop(Property_Path, Value) :-
    with_mutex(Property_Path,
               udp_set_prop_1(Property_Path, Value)).


%! udp_set_prop_1(+Property_Path, +Value) is det.

udp_set_prop_1(Property_Path, Value) :-
    functor(Fact, Property_Path, 1),
    (   \+ retract(Fact)   % Use \+ so we don't bind the value
    ->  existence_error(Property_Path, udp_output_property)
    ;   true
    ),
    arg(1, Fact, Value),
    assert(Fact).


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
    http_post(URL, json(#{path:Property_Path, type:Type, value:Value}), _, []),
    debug(swifg, '~q', [http_set_prop(Property_Path, Value)]).


%! user:capture_step_response(+PID_Id, +Elapsed_Time, -Control) is semidet.

user:capture_step_response(climb, Elapsed_Time, 0.05) :-
    flag(step_response_underway, Step_Response_Underway, Step_Response_Underway),
    (   Step_Response_Underway == 1
    ->  true
    ;   http_get_prop('/position/altitude-agl-ft', Altitude_AGL_Ft),
        Altitude_AGL_Ft > 500,
        debug(swifg, 'climb step response test started at ~ws', [Elapsed_Time]),
        flag(step_response_underway, _, 1)
    ).
