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

:- use_module(library(lists), [min_member/2, max_member/2, member/2]).
:- use_module(library(pce), [new/2, send/2]).
:- use_module(library(debug), [assertion/1]).

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
   sodt(2, +, +, +, +, +, +, +, +, -),
   sodt_1(+, 2, +, +, +, +, +, +, +, +, +, +, -).


%!  ts is semidet.

user:ts :-
   Duration = 1.0,
   Sampling_Frequency = 1000,
   b_a(B0, B1, B2, A0, A1, A2),
%   Input_Fn = step(0.0, 0.05),
   Input_Fn = sin(20),
   sodt(Input_Fn, Duration, Sampling_Frequency, B0, B1, B2, A0, A1, A2, Points),
   y_values(Points, Y_Values),
   min_member(Min, Y_Values),
   max_member(Max, Y_Values),
   format(string(Title), 'sodt - ~w', [b_a(B0, B1, B2, A0, A1, A2)]),
   new(W, auto_sized_picture(Title)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   Display_Duration is Duration / 10.0,
   send(Plotter, axis, plot_axis(x, 0.0, Display_Duration, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, Min, Max, @(default), 400)),
   send(Plotter, graph, new(X_Plot, plot_graph)),
   send(X_Plot, colour, green),
   send(Plotter, graph, new(Y_Plot, plot_graph)),
   send(Y_Plot, colour, blue),
   send(W, open),
   member(p(T, X, Y), Points),
   send(X_Plot, append, T, X),
   send(Y_Plot, append, T, Y),
   fail.

%!     b_a(B0, B1, B2, A0, A1, A2) is det.

b_a(0.06745527, 0.13491055, 0.06745527, 1, -1.1429805, 0.4128016).


%! step(+Time_Of_Step, +Step_Size, +Time, -X) is det.

step(Time_Of_Step, Step_Size, T, X) :-
   (   T =< Time_Of_Step
   ->  X = 0
   ;   X = Step_Size
   ).

%!  sin(+Frequency, +Time, -X) is det.
%
%  @arg Frequency Hz

sin(Frequency, Time, X) :-
   X is sin(2*pi * Frequency * Time).


%! y_values(+Points, -Y_Values) is det.

y_values([], []).
y_values([p(_, _, Y)|T1], [Y|T2]) :-
   y_values(T1, T2).


%!  sodt(+X_Pred, +Duration, +Sampling_Frequency, +B0, +B1, +B2,
%!       +A0, +A1, +A2, -Points) is det.
%
%  Applies a second-order linear discrete filter (direct form II transposed).
%
%  The general difference equation for a second-order filter is:
%
%  a0.y[n]=b0.x[n]+b1.x[n-1]+b2.x[n-2]-a1.y[n-1]-a2.y[n-2]
%
%  Assumes A0 is normalized to 1.
%
%  B0, B1, B2 are the numerator coefficients
%  A0, A1, A2 are the denominator coefficients
%
%  Reference
%  https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter.html
%
%  @arg X_Pred closure foo(..., Time, X)
%  @arg Y_Series list of p(T, X, Y)

sodt(X_Pred, Duration, Sampling_Frequency, B0, B1, B2, A0, A1, A2, Points) :-
   assertion(A0 == 1),
   N is integer(Duration * Sampling_Frequency),
   Time_Step is 1 / Sampling_Frequency,
   D1 = 0,
   D2 = 0,
   sodt_1(N, X_Pred, 0, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, Points).


%!  sodt_1(+K, +X_Pred, +Time, +Time_Step, +D1, +D2, +B0, +B1, +B2,
%         +A0, +A1, +A2, -Points) is det.
%
% From Python implementation:
%
% Difference equations for Direct Form II Transposed (second order)
%    y[n] = b0*x[n] + d1[n-1]
%    d1[n] = b1*x[n] - a1*y[n] + d2[n-1]
%    d2[n] = b2*x[n] - a2*y[n]
%
%    for n in range(len(x)):
%        y[n] = b[0] * x[n] + d1
%        d1 = b[1] * x[n] - a[1] * y[n] + d2
%        d2 = b[2] * x[n] - a[2] * y[n]

sodt_1(0, _, _, _, _, _, _, _, _, _, _, _, []) :-
   !.
sodt_1(K, X_Pred, T, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, [p(T, X, Y)|Points]) :-
    call(X_Pred, T, X),
    Y is B0 * X + D1,
    D1_New is B1 * X - A1 * Y + D2,
    D2_New is B2 * X - A2 * Y,
    KK is K-1,
    T_Next is T + Time_Step,
    sodt_1(KK, X_Pred, T_Next, Time_Step, D1_New, D2_New, B0, B1, B2, A0, A1, A2, Points).



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


