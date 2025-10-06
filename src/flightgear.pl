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

user:test :-
    process_create('/home/mike/Applications/flightgear-2024.1.2-linux-amd64.AppImage',
                   ['--httpd=8080', '--airport=NZWN', '--runway=34'],
                   [stderr(pipe(Out))]),
    repeat,
    read_line_to_string(Out, Line),
    sub_string(Line, _, _, _, "Primer reset to 0"),
    !,
    flightgear_http_connection(C),
    set_prop('/sim/current-view/view-number', 1, C),   % Chase view. Takes a few seconds to load
    align_heading_indicator,
    set_prop('/controls/engines/engine/throttle', 1.0, C),
    thread_create(steer_heading_on_ground(338), _, [detached(true)]),
    thread_create(fly_heading(340), _, [detached(true)]).


%! steer_heading_on_ground(+Heading:integer) is det.

steer_heading_on_ground(Heading) :-
    flightgear_http_connection(C),
    repeat,
    sleep(0.2),
    get_prop('/position/altitude-agl-ft', Altitude_AGL_Feet, C),
    Altitude_AGL_Feet < 10,
    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Indicated_Heading, C),
    direction_difference(Indicated_Heading, Heading, Error_Deg),   % +ve means right of intended track
    Normalised_Error is Error_Deg / 180.0,  % [-1, +1]
    clamped(-5.0 * Normalised_Error, -1, +1, Rudder),
    writeln(steering_on_ground(ih(Indicated_Heading), rudder(Rudder))),
    set_prop('/controls/flight/rudder', Rudder, C),
    fail.


%!  fly_heading(+Heading:integer) is det.

fly_heading(Heading) :-
    flightgear_http_connection(C),
    repeat,
    sleep(0.2),
    get_prop('/position/altitude-agl-ft', Altitude_AGL_Feet, C),
    Altitude_AGL_Feet >= 10,

    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Indicated_Heading, C),
    get_prop('/instrumentation/attitude-indicator/indicated-roll-deg', Indicated_Roll_Deg, C),
    direction_difference(Indicated_Heading, Heading, Heading_Error_Deg),    % +ve means right of intended track
    (   Heading_Error_Deg < -10
    ->  Required_Roll_Deg = 10

    ;   -10 =< Heading_Error_Deg, Heading_Error_Deg =< +10
    ->  Required_Roll_Deg is -Heading_Error_Deg

    ;   otherwise
    ->  Required_Roll_Deg = -10
    ),
    direction_difference(Indicated_Roll_Deg, Required_Roll_Deg, Roll_Error_Deg),  % +ve means roll left to null
    clamped(-10 * Roll_Error_Deg / 180.0, -0.2, +0.2, Aileron),

    writeln(fly_heading(ih(Indicated_Heading), he(Heading_Error_Deg), rrd(Required_Roll_Deg), ird(Indicated_Roll_Deg), aileron(Aileron))),
    set_prop('/controls/flight/aileron', Aileron, C),
    fail.


%!  align_heading_indicator is det.
%
%   Align the DI with the magnetic compass

align_heading_indicator :-
    flightgear_http_connection(C),
    get_prop('/instrumentation/magnetic-compass/indicated-heading-deg', Magnetic_Compass_Indicated_Heading, C),
    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Unaligned_Indicated_Heading, C),
    get_prop('/instrumentation/heading-indicator/align-deg', Initial_Align_Deg, C),
    direction_difference(Magnetic_Compass_Indicated_Heading, Unaligned_Indicated_Heading, Heading_Indicator_Alignment_Error),
    Align_Deg is integer(Initial_Align_Deg + Heading_Indicator_Alignment_Error) mod 360,
    set_prop('/instrumentation/heading-indicator/align-deg', Align_Deg, C).


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


%!  set_prop(+Property_Path, +Value, +HTTP_Connection) is det.

set_prop(Property_Path, Value, HTTP_Connection) :-
    py_call(HTTP_Connection:set_prop(Property_Path, Value)).

%!  get_prop(+Property_Path, -Value, +HTTP_Connection) is det.

get_prop(Property_Path, Value, HTTP_Connection) :-
    py_call(HTTP_Connection:get_prop(Property_Path), Value).





    /*
     *
     *
    set_prop('/position/altitude-ft', 2000, C),

    set_prop('/controls/engines/current-engine/carb-heat', 0, C),
    set_prop('/controls/engines/engine/mixture', 0.6, C),
    set_prop('/controls/engines/engine/throttle', 0.8, C),

    set_prop('/controls/switches/starter', true, C),
    sleep(5.0),
    set_prop('/controls/switches/starter', false, C),


    get_prop('/position/altitude-ft', A0, C),
    between(0, 10, I),
    A1 is A0 + I * 100,
    set_prop('/position/altitude-ft', A1, C),
    fail.
*/
