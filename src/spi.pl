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
:- use_module(library(clpfd)).

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
   /*
   Duration = 1.0,
   Sampling_Frequency = 1000,
%  Input_Fn = step(0.0, 0.05),
%  Input_Fn = sin(20),
%  Display_Duration is Duration / 10.0,
*/
   Duration = 200,
   Sampling_Frequency = 10,  % Hz
   Display_Duration = Duration,
   step_response(dat('step_response.dat'), Measured_Step_Response),
   Input_Fn = step(0.0, 0.05),

   b_a(B0, B1, B2, A0, A1, A2),
   sodt(Input_Fn, Duration, Sampling_Frequency, B0, B1, B2, A0, A1, A2, SODT_Points),
   append(SODT_Points, Measured_Step_Response, All_Points),
   y_values(All_Points, Y_Values),
   min_member(Min, Y_Values),
   max_member(Max, Y_Values),
   format(string(Title), 'sodt - ~w', [b_a(B0, B1, B2, A0, A1, A2)]),
   new(W, auto_sized_picture(Title)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   send(Plotter, axis, plot_axis(x, 0.0, Display_Duration, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, Min, Max, @(default), 400)),
   send(Plotter, graph, new(Blue_Plot, plot_graph)),
   send(Blue_Plot, colour, blue),
   send(Plotter, graph, new(Black_Plot, plot_graph)),
   send(Black_Plot, colour, black),
   legend([item('Measured Step Response', black), item('Modelled Step Response', blue)], Plotter, 30, 80),
   send(W, open),
   forall(member(p(T, Y), SODT_Points), send(Blue_Plot, append, T, Y)),
   forall(member(p(T, M), Measured_Step_Response), send(Black_Plot, append, T, M)),
   fail.


%! legend(+Items, +Plotter, +Y, +Spacing) is det.

legend(Items, Plotter, Y, Spacing) :-
   get(Plotter, width, Plotter_Width),
   legend_1(Items, Plotter, X_Left, Y, Spacing, 0, Total_Width),
   Left_Margin #= X_Left,
   Slop in 0..1,         % Because Plotter_Width - Total_Width may be odd
   Right_Margin #= Left_Margin + Slop,
   Left_Margin + Total_Width + Right_Margin #= Plotter_Width,  % Center the legend
   once(labeling([min(Slop)], [Slop])).


legend_1([], _, _, _, _, W, W).
legend_1([item(Legend_Text, Colour)|T], Plotter, X, Y, Spacing, Width, Total_Width) :-
   format(atom(Prefixed_Text), '--------- ~w', [Legend_Text]),
   new(Text, text(Prefixed_Text)),
   send(Text, colour, Colour),
   get(Text, width, Text_Width),
   freeze(X, send(Plotter, display, Text, point(X, Y))),
   Next_X #= X + Text_Width + Spacing,
   New_Width #= Width + Text_Width + Spacing,
   legend_1(T, Plotter, Next_X, Y, Spacing, New_Width, Total_Width).


%!  b_a(B0, B1, B2, A0, A1, A2) is det.

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


%! step_response(+File_Name, -Step_Response) is det.
%
%  @arg Step_Response list p(T, X)

:- dynamic
   p/2.

step_response(File_Name, Step_Response) :-
   consult(File_Name),
   findall(p(T, X), p(T, X), L),
   [p(T0, _)|_] = L,
   rebase_time(L, T0, Step_Response).


%!  rebase_time(+L1, +T0, -L2) is det.

rebase_time([], _, []).
rebase_time([p(T, V)|T1], T0, [p(T_Rebased, V)|T2]) :-
    T_Rebased is T-T0,
    rebase_time(T1, T0, T2).


%! measured_step_response(+Step_Response, +Time, -X) is det.
%
%  @arg Step_Response list of p(T, X)

measured_step_response(Step_Response, Time, X) :-
   nextto(p(T1, X0), p(T2, _), Step_Response),
   T1 =< Time, Time < T2,
   !,
   X = X0.


%! y_values(+Points, -Y_Values) is det.

y_values([], []).
y_values([p(_, Y)|T1], [Y|T2]) :-
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
%  @arg Y_Series list of p(T, Y)

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
sodt_1(K, X_Pred, T, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, [p(T, Y)|Points]) :-
    call(X_Pred, T, X),
    Y is B0 * X + D1,
    D1_New is B1 * X - A1 * Y + D2,
    D2_New is B2 * X - A2 * Y,
    KK is K-1,
    T_Next is T + Time_Step,
    sodt_1(KK, X_Pred, T_Next, Time_Step, D1_New, D2_New, B0, B1, B2, A0, A1, A2, Points).


