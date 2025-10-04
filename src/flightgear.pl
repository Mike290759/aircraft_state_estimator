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

% ~/Applications/flightgear-2024.1.2-linux-amd64.AppImage --httpd=8080 --airport=NZWN --runway=34
user:test :-
    py_call('flightgear_python.fg_if':'HTTPConnection'(localhost, 8080), C),
    % py_call(C:list_props('/position', recurse_limit=0), Props), py_call(pprint:pprint(Props)),
    % py_call(C:list_props('/controls', recurse_limit=0), Props), py_call(pprint:pprint(Props)),

    repeat,
    set_prop('/instrumentation/heading-indicator/align-deg', 294, C),
    get_prop('/instrumentation/heading-indicator/indicated-heading-deg', Heading, C),
    Error_Degrees is integer((340 - Heading + 540)) mod 360 - 180,
    Normalised_Error is Error_Degrees / 180.0,  % [-1, +1]
    clamped(2.0 * Normalised_Error, -1, +1, Rudder),
    writeln([Heading, Rudder]),
    set_prop('/controls/flight/rudder', Rudder, C),
    sleep(0.5),
    fail.

clamped(Expr, Left, Right, Clamped) :-
    X is Expr,
    (   X =< Left
    ->  Clamped = Left
    ;   X >= Right
    ->  Clamped = Right
    ;   Clamped = X
    ).


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

%!  set_prop(+Property_Path, +Value, +HTTP_Connection) is det.

set_prop(Property_Path, Value, HTTP_Connection) :-
    py_call(HTTP_Connection:set_prop(Property_Path, Value)).

%!  get_prop(+Property_Path, -Value, +HTTP_Connection) is det.

get_prop(Property_Path, Value, HTTP_Connection) :-
    py_call(HTTP_Connection:get_prop(Property_Path), Value).

