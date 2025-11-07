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

:- use_module(library(lists), [min_member/2, max_member/2, member/2, append/3]).
:- use_module(library(pce), [new/2, send/2, get/3]).
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
   sodt_1(+, 2, +, +, +, +, +, +, +, +, +, +, -),
   galg(2, +, -, -).


%!  ts is semidet.

user:ts :-
   Duration = 155,  % Duration of the measured response we want to consider
   measured_open_loop_step_response(dat('step_response.dat'), Duration, Sample_Time, Step_Size, Measured_Open_Loop_Step_Response),
   Initial_Genes = [0.5, 0.5, 0.5, 0.5, 0.5],
   galg(fitness(Measured_Open_Loop_Step_Response, Sample_Time, Step_Size, Modelled_Open_Loop_Step_Response), Initial_Genes, Better_Genes, _Fitness),
   append(Modelled_Open_Loop_Step_Response, Measured_Open_Loop_Step_Response, All_Points),
   y_values(All_Points, Y_Values),
   min_member(Min, Y_Values),
   max_member(Max, Y_Values),
   format(string(Title), 'sodt - ~w', [Better_Genes]),
   new(W, auto_sized_picture(Title)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   send(Plotter, axis, plot_axis(x, 0.0, Duration, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, Min, Max, @(default), 400)),
   send(Plotter, graph, new(Blue_Plot, plot_graph)),
   send(Blue_Plot, colour, blue),
   send(Plotter, graph, new(Black_Plot, plot_graph)),
   send(Black_Plot, colour, black),
   legend([item('Measured Open Loop Step Response', black), item('Modelled Open Loop Step Response', blue)], Plotter, 30, 80),
   send(W, open),
   forall(member(p(T, Y), Modelled_Open_Loop_Step_Response), send(Blue_Plot, append, T, Y)),
   forall(member(p(T, M), Measured_Open_Loop_Step_Response), send(Black_Plot, append, T, M)),
   fail.


%! legend(+Items, +Plotter, +Y, +Spacing) is det.

legend(Items, Plotter, Y, Spacing) :-
   get(Plotter, width, Plotter_Width),
   legend_1(Items, Plotter, X_Left, Y, Spacing, 0, Total_Width),
   Left_Margin in 0..Total_Width,
   Right_Margin in 0..Total_Width,
   Left_Margin = X_Left,
   Slop in 0..1,         % Because Plotter_Width - Total_Width may be odd
   Right_Margin #= Left_Margin + Slop,
   Left_Margin + Total_Width + Right_Margin #= Plotter_Width,  % Center the legend
   once(labeling([min(Slop)], [Slop, Left_Margin, Right_Margin])).


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


%! step(+Time_Of_Step, +Step_Size, +Time, -X) is det.

step(Time_Of_Step, Step_Size, T, X) :-
   (   T =< Time_Of_Step
   ->  X = 0
   ;   X = Step_Size
   ).


%! measured_open_loop_step_response(+File_Name, +Duration, -Sample_Time,
%!                                  -Step_Size, -Step_Response) is
%!                                  det.
%
%  Supply the data captured in the PID loop - see capture_open_loop_step_response/3
%
%  @arg Duration cutoff time for the number of data points to be returned
%  @arg Sample_Time the length of time between measurements
%  @arg Step_Size the size of input step
%  @arg N the number of data points in the step response series
%  @arg Step_Response list p(T, V)

measured_open_loop_step_response(File_Name, Duration, Sample_Time, Step_Size, Step_Response) :-
   absolute_file_name(File_Name, Absolute_File_Name),   % Allow for path aliases
   setup_call_cleanup(open(Absolute_File_Name, read, In),
                      read_step_response(In, Sample_Time, Step_Size, L),
                      close(In)),
   rebase_time_and_trim(L, Duration, Step_Response).


%!  read_step_response(+In:stream, -Sample_Time, -Step_Size, -Step_Response) is det.

read_step_response(In, Sample_Time, Step_Size, Step_Response) :-
   read_term(In, Sample_Time_Term, []),   % Must be the first term in file
   (   Sample_Time_Term = sample_time(Sample_Time)
   ->  true
   ;   throw(sample_time_expected)
   ),
   read_term(In, Step_Size_Term, []),   % Must be the second term in file
   (   Step_Size_Term = step_size(Step_Size)
   ->  true
   ;   throw(step_size_expected)
   ),
   findall(p(T, V),
           point(In, T, V),
           Step_Response).


%! point(+In:stream, -T, -V) is det.

point(In, T, V) :-
   repeat,
   read_term(In, Term, []),
   (   Term == end_of_file
   ->  !,
       fail

   ;   Term = p(T, V)
   ->  true

   ;   !,
       fail
   ).


%!  rebase_time_and_trim(+L1, +Duration, -L2) is det.
%
%  Make the time of the first item 0.0 and adjust remaining points
%  accordingly.
%
%  Drop points more than Duration after the first point

rebase_time_and_trim(L1, Duration, L2) :-
   L1 = [p(T0, _)|_],
   T_Cutoff is T0 + Duration,
   rebase_time_and_trim_1(L1, T0, T_Cutoff, L2).


%!  rebase_time_and_trim(+L1, +T0, +T_Cutoff, -L2) is det.

rebase_time_and_trim_1([], _, _, []).
rebase_time_and_trim_1([p(T, _)|_], _, T_Cutoff, []) :-
   T > T_Cutoff,
   !.
rebase_time_and_trim_1([p(T, V)|L1], T0, T_Cutoff, [p(T1, V)|L2]) :-
    T1 is T-T0,
    rebase_time_and_trim_1(L1, T0, T_Cutoff, L2).


%! y_values(+Points, -Y_Values) is det.

y_values([], []).
y_values([p(_, Y)|T1], [Y|T2]) :-
   y_values(T1, T2).


%!  sodt(+X_Pred, +N, +Sampling_Frequency, +B0, +B1, +B2,
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
%  @arg N series length
%  @arg X_Pred closure foo(..., Time, X)
%  @arg Y_Series list of p(T, Y)

sodt(X_Pred, N, Sample_Time, B0, B1, B2, A0, A1, A2, Points) :-
   assertion(A0 == 1),
   Time_Step = Sample_Time,
   D1 = 0,
   D2 = 0,
   sodt_1(N, X_Pred, 0, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, Points).


%!  sodt_1(+K, +X_Pred, +Time, +Time_Step, +D1, +D2, +B0, +B1, +B2, +A0, +A1, +A2, -Points) is det.
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


%! fitness(+Measured_Open_Loop_Step_Response, +Sample_Time, +Step_Size,
%!         -Modelled_Open_Loop_Step_Response, +Genes, -Fitness) is det.

fitness(Measured_Open_Loop_Step_Response, Sample_Time, Step_Size, Modelled_Open_Loop_Step_Response, [B0, B1, B2, A1, A2], Fitness) :-
   length(Measured_Open_Loop_Step_Response, N),
   sodt(step(0.0, Step_Size), N, Sample_Time, B0, B1, B2, 1, A1, A2, Modelled_Open_Loop_Step_Response),
   fitness_1(Measured_Open_Loop_Step_Response, Modelled_Open_Loop_Step_Response, 0, Fitness).


%! fitness_1(+Measured_Open_Loop_Step_Response,
%!           +Modelled_Open_Loop_Step_Response, +Fitness_Accum,
%!           -Fitness) is det.

fitness_1([], [], Fitness, Fitness).
fitness_1([p(_, V1)|T1], [p(_, V2)|T2], F0, F) :-
   F1 is F0 + (V1-V2) ** 2,
   fitness_1(T1, T2, F1, F).



%!  galg(+Fitness_Pred, +Initial_Genes, -Better_Genes, -Fitness) is det.

galg(Fitness_Pred, Genes, Better_Genes, Fitness) :-
   call(Fitness_Pred, Genes, Fitness),
   Better_Genes = Genes.


%! value_to_gene(+Value, -Bits) is det.
%
%  A two's complement representation is used.
%
%  Positive numbers:
%    The binary representation is the same as its unsigned counterpart.
%
%  Negative numbers:
%    The two's complement is found by taking the
%    positive version, inverting all its bits, and then adding 1
%    ignoring any overflow
%
%  Example with 16 bits and range of -100.0..100.0:
%    -100.0 --> ???
%       0.0 -->
%    +100.0 -->
%
%  @arg Value float @arg Bits list of 1's and 0's [Sign_Bit, MSB, ... LSB]

value_to_gene(Value, Bits) :-
   gene_width(Width),
   gene_value_range(Range),
   assertion((-Range =< Value, Value =< Range)),
   Abs_Value is abs(Value),
   format('Abs_Value=~f~n', [Abs_Value]),
   I0 is integer((1<<(Width-1)-1) * Abs_Value/Range),
   format('I0=~2r~n', [I0]),
   (   Value < 0
   ->  I1 is I0 + 1,
       I is I1 /\ ((1<<Width)-1)   % One's complement plus 1, ignore overflow

   ;   I = I0
   ),
   format('I=~2r [~d]~n', [I,I]),
   int_to_bits(I, Width, Bits).


%! int_to_bits(+I, +K, -Encoding) is det.

int_to_bits(_, 0, []) :-
   !.
int_to_bits(I, K, [Bit|T]) :-
   KK is K - 1,
   Bit is getbit(I, KK),
   int_to_bits(I, KK, T).


gene_width(16).
gene_value_range(100.0).  % Range is -100.0 .. 100.0


%! gene_to_value(+Encoding, -Value) is det.
%
%  @arg Value float
%  @arg Encoding list of 1's and 0's with MSB on the left

gene_to_value([Sign_Bit|Bits], Value) :-
   writeln(bits(Bits)),
   bits_to_int(Bits, 0, I),
   writeln(i(I)),
   gene_value_range(Range),
   gene_width(Width),
   sign(Sign_Bit, Sign),
   Value is Sign * Range * I/(1<<Width).


%! sign(+Sign_Bit, -Sign) is det.

sign(1, -1).
sign(0, 1).


%! bits_to_int(+Encoding, +Accum, -I) is det.

bits_to_int([], Accum, Accum).
bits_to_int([H|T], Accum, I) :-
   Accum_1 is Accum * 2 + H,
   writeln(x(Accum_1)),
   bits_to_int(T, Accum_1, I).

