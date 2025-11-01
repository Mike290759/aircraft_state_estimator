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

%!  ts is semidet.

user:ts :-
   N = 500,
   step_series(N, 0.1, [p(_, X2), p(_, X1)|X]),
   sodt(X, Y, X2, X1, 0, 0, 0, 1, 0.1, 1.1, 1.2, 1.0),
   new(W, auto_sized_picture(sodt)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   send(Plotter, axis, plot_axis(x, 0.0, N, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, 0.0, 2.0, @(default), 400)),
   send(Plotter, graph, new(Plot, plot_graph)),
   send(Plot, colour, green),
   send(W, open),
   member(p(T, V), Y),
   send(Plot, append, T, V),
   sleep(0.01),
   fail.


%!  sodt(+X, -Y, +X2, +X1, +Y2, +Y1, +I, +A1, +A2, +B0, +B1, +B2)
%!       is det.
%
%  X1 means x[k-1], X2 means x[k-1] etc, same for Y1 etc
%
%  Second-order discrete-time linear system
%  Difference Equation: A second-order discrete-time linear system can be represented by the
%  following general form of a difference equation:
%
%  y[k]+a1.y[k-1]+a2.y[k-2] = b0.x[k]+b1.x[k-1]+b2.x[k-2]
%
%  H(Z) = B0 + B1 * Z ^ -1 + B2 * Z ^ -2 / 1 + A1 * Z ^ -1 + A2 * Z ^ -2
%
%  @arg X list of p(T, V)
%  @arg Y list of p(T, V)

sodt([], [], _, _, _, _, _, _, _, _, _, _).
sodt([p(T0, X0)|L1], [p(T0, Y0)|L2], X2, X1, Y2, Y1, I, A1, A2, B0, B1, B2) :-
    Y0 is B0 * X0 + B1 * X1 + B2 * X2 - A1 * Y1 - A2 * Y2,
    II is I + 1,
    sodt(L1, L2, X1, X0, Y1, Y0, II, A1, A2, B0, B1, B2).


%! step_series(+Number_Of_Steps, +Time_Step, -Step_Series: list) is det.
%
%   @arg Step_Series list of p(Time, Value)

step_series(N, Time_Step, [p(0.0, 0.0), p(Time_Step, 0.0)|T]) :-
   N1 is N-2,
   Time is 2 * Time_Step,
   step_series_1(N1, Time_Step, Time, T).

step_series_1(0, _, _, []) :-
   !.
step_series_1(N, Time_Step, Time, [p(Time, 1.0)|T]) :-
   NN is N-1,
   Time_1 is Time + Time_Step,
   step_series_1(NN, Time_Step, Time_1, T).


%  fit is det.

:- dynamic p/2.

user:fit :-
    consult(dat('step_response.dat')),
    findall(p(T,V), p(T,V), L1),
    L1 = [p(T0, _)|_],
    rebase_time(L1, T0, L2),
    writeln(L2).


%!  rebase_time(+L1, +T0, -L2) is det.

rebase_time([], _, []).
rebase_time([p(T, V)|T1], T0, [p(T_Rebased, V)|T2]) :-
    T_Rebased is T-T0,
    rebase_time(T1, T0, T2).


