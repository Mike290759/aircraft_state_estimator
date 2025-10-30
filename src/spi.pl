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

:- module(spi,
          [
          ]).

/** <module> System Parameter Identification

A second-order discrete-time linear system is a mathematical model that describes
a system's behavior using a second-order difference equation, where the current output
depends on the two previous inputs and the two previous outputs.

These systems are widely used in engineering to model dynamic processes and are characterized by a
transfer function with a second-order polynomial in the denominator, and their stability
depends on the location of the system's poles within the unit circle in the z-plane.

See also https://courses.cs.duke.edu/spring07/cps111/notes/03.pdf
*/

%!  sodt(+In:stream, +Out:stream, +A1, +A2, +B0, +B1, +B2) is
%!       det.
%
%   Second-order discrete-time linear system
%   Difference Equation: A second-order discrete-time linear system can be represented by the
%   following general form of a difference equation:
%
%   y[k]+a1.y[k-1]+a2.y[k-2] = b0.u[k]+b1.u[k-1]+b2.u[k-2]
%
%   H(Z) = B0 + B1 * Z ^ -1 + B2 * Z ^ -2 / 1 + A1 * Z ^ -1 + A2 * Z ^ -2

sodt(In, Out, A1, A2, B0, B1, B2) :-
   read(In, X2),
   X2 \== end_of_file,
   read(In, X1),
   X1 \== end_of_file,
   sodt_1(In, Out, 2, A1, A2, B0, B1, B2, X2, X1, 0, 0).

sodt_1(In, Out, T, A1, A2, B0, B1, B2, X2, X1, Y2, Y1) :-
    read(In, X0),
    X0 \== end_of_file,
    !,
    Y0 is B0 * X0 + B1 * X1 + B2 * X2 - A1 * Y1 + A2 * Y2,
    format(Out, '~q.~n', [p(T,Y0)]),
    TT is T + 1,
    sodt_1(In, Out, TT, A1, A2, B0, B1, B2, X1, X0, Y1, Y0).
sodt_1(_, _, _, _, _, _, _, _, _, _, _, _).

user:ts :-
    setup_call_cleanup(open('step.dat', read, In),
                       setup_call_cleanup(open('step_response.dat', write, Out),
                                          sodt(In, Out, 1, 0.5, 1.1, 1.2, 1.0),
                                          close(Out)),
                       close(In)),
    new(W, auto_sized_picture(sodt)),
    send(W, max_size, size(2000, 600)),
    send(W, display, new(Plotter, plotter)),
    send(Plotter, axis, plot_axis(x, 0, 1000, @(default), 1500)),
    send(Plotter, axis, plot_axis(y, -1000.0, 1000.0, @(default), 400)),
    send(Plotter, graph, new(Plot, plot_graph)),
    send(Plot, colour, green),
    send(W, open),
    setup_call_cleanup(open('step_response.dat', read, In_1),
                       (   repeat,
                           read(In_1, Term),
                           (   Term == end_of_file
                           ->  !
                           ;   Term = p(T,Y),
                               -1000 =< Y, Y =< 1000,
                               writeln([T, Y]),
                               send(Plot, append, T, Y),
                               fail
                           )),
                       close(In_1)).


user:cs :-
    setup_call_cleanup(open('step.dat', write, S),
                       cs_1(S),
                       close(S)).

cs_1(S) :-
    between(1, 1000, I),
    (   I =< 500
    ->  Y = 0
    ;   Y = 1
    ),
    format(S, '~q.~n', [Y]),
    fail.
cs_1(_).
