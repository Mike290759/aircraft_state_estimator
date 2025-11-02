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

:- meta_predicate
   sodt(2, -, +, +, +, +, +, +, +),
   sodt_1(+, 2, +, +, +, +, +, +, +, +, +, +, +).


%!  ts is semidet.

user:ts :-
   N = 100,
   sodt(step(0.01, 0.05), Y, 0, N, 0.1, 0.99, 0.04, 1, 0),
   new(W, auto_sized_picture(sodt)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   send(Plotter, axis, plot_axis(x, 0.0, N, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, 0.0, 0.05, @(default), 400)),
   send(Plotter, graph, new(Plot, plot_graph)),
   send(Plot, colour, green),
   send(W, open),
   member(p(T, V), Y),
   writeln((T,V)),
   T_Plot is T * 500,
   send(Plot, append, T_Plot, V),
   sleep(0.01),
   fail.

%! step(+Time_Of_Step, +Step_Size, +Time, -X) is det.

step(Time_Of_Step, Step_Size, T, X) :-
   (   T =< Time_Of_Step
   ->  X = 0
   ;   X = Step_Size
   ).


%!  sodt(+X_Pred, -Y_Series, +Time, +N, +Time_Step, +A1:number,
%!       +A2:number, +B1:number, +B2:number) is det.
%
%  Second-order discrete-time linear system
%  Difference Equation: A second-order discrete-time linear system can be represented by the
%  following general form of a difference equation:
%
%  y[k]+a1.y[k-1]+a2.y[k-2] = b1.x[k-1]+b2.x[k-2]

%  @arg X_Pred closure foo(..., Time, X)
%  @arg Y_Series list of p(T, V)

sodt(X_Pred, Y_Series, Time, N, Time_Step, A1, A2, B1, B2) :-
   sodt_1(N, X_Pred, Y_Series, Time, Time_Step, 0, 0, 0, 0, A1, A2, B1, B2).


%!  sodt_1(+K, +X_Pred, -Y_Series, +Time, +Time_Step, +X2, +X1, +Y2,
%!         +Y1, +A1, +A2, +B1, +B2) is det.
%  X1 means x[k-1], X2 means x[k-1] etc, same for Y1 etc

sodt_1(0, _, [], _, _, _, _, _, _, _, _, _, _) :-
   !.
sodt_1(K, X_Pred, [p(T, Y0)|Y_Series], T, Time_Step, X2, X1, Y2, Y1, A1, A2, B1, B2) :-
    call(X_Pred, T, X0),
    Y0 is B1 * X1 + B2 * X2 - A1 * Y1 - A2 * Y2,
    KK is K-1,
    T_Next is T + Time_Step,
    sodt_1(KK, X_Pred, Y_Series, T_Next, Time_Step, X1, X0, Y1, Y0, A1, A2, B1, B2).



%  fit is det.

:- dynamic p/2.
/*
user:fit :-
   N = 5,
   consult(dat('step_response.dat')),
   once(findnsols(N, p(T,V), p(T,V), L)),
   L = [p(T0, _)|_],
   rebase_time(L, T0, Measured_Step_Response),
   step_series
   sodt(X, Y, A1, A2, B0, B1, B2),
   writeln(X),
   writeln(Y).
*/
%!  rebase_time(+L1, +T0, -L2) is det.

rebase_time([], _, []).
rebase_time([p(T, V)|T1], T0, [p(T_Rebased, V)|T2]) :-
    T_Rebased is T-T0,
    rebase_time(T1, T0, T2).


