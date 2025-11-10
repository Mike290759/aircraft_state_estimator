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

%:- set_prolog_flag(optimise, true).

%!  ts is semidet.

user:ts :-
   Duration = 155,  % Duration of the measured response we want to consider
   measured_open_loop_step_response(dat('step_response.dat'), Duration, Sample_Time, Step_Size, Measured_Open_Loop_Step_Response),
   galg(fitness(Measured_Open_Loop_Step_Response, Sample_Time, Step_Size, Modelled_Open_Loop_Step_Response), Fittest_Gene),
   append(Modelled_Open_Loop_Step_Response, Measured_Open_Loop_Step_Response, All_Points),
   y_values(All_Points, Y_Values),
   min_member(Min, Y_Values),
   max_member(Max, Y_Values),
   format(string(Title), 'sodt - ~w', [Fittest_Gene]),
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
   read_term(In, Step_Size_Term, []),   % Must be the second term in file
   (   Step_Size_Term = step_size(Step_Size)
   ->  true
   ;   throw(step_size_expected)
   ),
   read_term(In, Sample_Time_Term, []),   % Must be the first term in file
   (   Sample_Time_Term = sample_time(Sample_Time)
   ->  true
   ;   throw(sample_time_expected)
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



%!  galg(+Fitness_Pred, -Fittest_Gene) is det.

:- det(galg/2).

galg(Fitness_Pred, Fittest_Gene) :-
   galg_config(C),
   galg_config(C),
   Chromosome_Length is C.number_of_parameters * C.parameter_bit_width,
   initial_population(C.population_size, Fitness_Pred, Chromosome_Length, Initial_Population),
   galg_1(0, C.max_generations, Fitness_Pred, Initial_Population, Final_Population),
   fittest(Final_Population, Fittest_Gene).


%! galg_1(+Generation, +Max_Generations, +Fitness_Pred,
%!        +Population_In, -Population_Out) is det.
%
%  A Population is a list of terms individual(Fitness, Chromosome) where
%  Chromosome is a list of bits being a concatenation of fixed width
%  lists of bits (genes).

:- det(galg_1/6).

galg_1(Generation, Max_Generations, _, Population, Population) :-
   Generation >= Max_Generations,
   !.
galg_1(G, Max_Generations, Fitness_Pred, P0, Population_Out) :-
   reproduce(P0, P1),
   swap_genes(P1, P2),
   mutate(P2, P3),
   !,  % Green cut
   GG is G + 1,
   galg_1(GG, Max_Generations, Fitness_Pred, P3, Population_Out).


%!  initial_population(+Population_Size, +Fitness_Pred, -Population) is
%!                     det.

:- det(initial_population/4).

initial_population(0, _, _, []) :-
   !.
initial_population(N, Fitness_Pred, Chromosome_Length, [individual(Fitness, Chromosome)|P]) :-
   initial_chromosome(Chromosome_Length, Chromosome),
   evaluate_fitness(Chromosome, Fitness_Pred, Fitness),
   NN is N - 1,
   initial_population(NN, Fitness_Pred, P).


%!  initial_chromosome(+N, -Chromosome) is det.

:- det(initial_chromosome/2).

initial_chromosome(0, []) :-
   !.
initial_chromosome(N, [Bit|Bits]) :-
   random_between(0, 1, Bit),
   NN is N - 1,
   initial_chromosome(NN, Bits).



%! evaluate_fitness(+Chromosome, +Fitness_Pred, -Fitness) is det.

:- det(evaluate_fitness/3).

evaluate_fitness(Chromosome, Fitness_Pred, Fitness) :-
   galg_config(C),
   chromosome_genes(Chromosome, C.parameter_bit_width, Genes),
   call(Fitness_Pred, Genes, Fitness).


%!  chromosome_genes(+Chromosome, +Width, -Genes) is det.

:- det(chromosome_genes/3).

chromosome_genes([], _, []) :-
   !.
chromosome_genes(Chromosome, Width, [Gene|Genes]) :-
   length(Gene, Width),
   append(Gene, Chromosome_Rest, Chromosome),
   chromosome_genes(Chromosome_Rest, Width, Genes).


%! reproduce(+Population, -New_Population) is det.
%
%  Reproduce in proportion to fitness

reproduce(P0, P2) :-
   sort(1, @>=, P0, P1),
   total_fitness(P1, 0, Total_Fitness),
   galg_config(C),
   phrase(reproduce_1(P1, Total_Fitness, 0, C.population_size), P2).


%! reproduce_1(+Population, +Total_Fitness, +Population_Count,
%!             +Population_Size) // is det.

reproduce_1(_, _, N, Population_Size) -->
   { N >= Population_Size
   },
   !,
   [].
reproduce_1([individual(Fitness, Chromosome)|T], Total_Fitness, N, Population_Size) -->
   { Number_Required is integer(Population_Size * Fitness/Total_Fitness),
     Number_Required > 0,
     !,
     NN is N + Number_Required
   },
   clones(Number_Required, individual(Fitness, Chromosome)),
   reproduce_1(T, Total_Fitness, NN, Population_Size).
reproduce_1([Individual|T], Total_Fitness, N, Population_Size) -->
   [Individual],
   { NN is N + 1
   },
   reproduce_1(T, Total_Fitness, NN, Population_Size).


%! clones(+Number_Required, +Individual) // is det.

clones(0, _) -->
   !,
   [].
clones(N, Individual) -->
   [Individual],
   { NN is N - 1
   },
   clones(NN, Individual).



%! total_fitness(+Population, +Accum, -Total_Fitness) is det.

total_fitness([], Accum, Accum).
total_fitness([individual(F, _)|T], Accum, Fitness) :-
   New_Accum is Accum + F,
   total_fitness(T, New_Accum, Fitness).


%! swap_genes(+Population) // is det.

swap_genes([]) -->
   !,
   [].
swap_genes(P0) -->
   { random_select(individual(_, C1), P0, P1),
     random_select(individual(_, C2), P1, P2),
     length(C1, Chromosome_Length),
     random_between(1, Chromosome_Length, I),
     length(C1_Left, I),
     length(C2_Left, I),
     append(C1_Left, C1_Right, C1),
     append(C1_Left, C2_Right, C3),
     append(C2_Left, C1_Right, C4),
     append(C2_Left, C2_Right, C2)
   },
   [individual(_, C3), individual(_, C4)],
   swap_genes(P2).




test_pop([individual(70, a), individual(40, a), individual(30, a), individual(8, a), individual(7, a), individual(6, a), individual(5, a), individual(4, a), individual(3, a), individual(2, a)]).


%! galg_config(-Dict) is det.

galg_config(galg_config{number_of_parameters: 5,
                        parameter_bit_width: 16,
                        population_size: 10,
                        retention: 10,
                        max_generations: 100}).

%! value_to_gene(+Value, +Width, +Range, -Bits) is det.
%
%  A two's complement representation is used to represent all numbers
%  positive and negative.
%
%  Positive numbers:
%    The MSB is 0
%    The biggest value in 16-bits is 0111_1111_1111_1111 = 32,767
%
%  Negative numbers:
%    With the two's complement representation the MSB is
%    treated as -(2^Width-1) and the bits to the right are added to give
%    a negative number. For example in a 16 bit system:
%
%    1000_0000_0000_0000 is -2^15 = -32768 (the biggest negative value)
%    1000_0000_0000_0010 is -2^15 + 2 = -32766
%    1111_1111_1111_1111 is -2^15 + 2^15 - 1 = -1
%
%  Example with 16 bits and range of -100.0..100.0:
%    -100.0 --> [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
%       0.0 --> [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
%    +100.0 --> [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
%
%  @arg Value float
%  @arg Width integer number of bits in the gene
%  @arg Range float the max/min limit of Value
%  @arg Bits list of 1's and 0's [Sign_Bit, MSB, ... LSB]

:- det(value_to_gene/4).

value_to_gene(Value, Width, Range, [Sign_Bit|Bits]) :-
   assertion((-Range =< Value, Value =< Range)),
   NV is Value/Range,
   (   NV < 0
   ->  I is 1<<(Width-1) + integer(1<<(Width-1) * NV),
       Sign_Bit = 1

   ;   I is integer((1<<(Width-1)-1) * NV),
       Sign_Bit = 0
   ),
   int_to_bits(I, Width-1, Bits).


%! int_to_bits(+I, +K, -Encoding) is det.

int_to_bits(_, 0, []) :-
   !.
int_to_bits(I, K, [Bit|T]) :-
   KK is K - 1,
   Bit is getbit(I, KK),
   int_to_bits(I, KK, T).



%! gene_to_value(+Encoding, +Range, -Value) is det.
%
%  Inverse of value_to_gene/4
%
%  @arg Value float
%  @arg Bits list of 1's and 0's [Sign_Bit, MSB, ... LSB]

:- det(gene_to_value/3).

gene_to_value([Sign_Bit|Bits], Range, Value) :-
   length([Sign_Bit|Bits], Width),
   bits_to_int(Bits, 0, I),
   (   Sign_Bit == 1
   ->  Neg_Offset is -(1<<(Width-1))
   ;   Neg_Offset = 0
   ),
   Value is Range * (Neg_Offset + I)/(1<<(Width-1)).


%! bits_to_int(+Encoding, +Accum, -I) is det.

bits_to_int([], Accum, Accum).
bits_to_int([H|T], Accum, I) :-
   Accum_1 is Accum * 2 + H,
   bits_to_int(T, Accum_1, I).


:- begin_tests(spi).

test(1) :-
   value_to_gene(-100.0, 16, 100.0, L),
   L == [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0].

test(2) :-
   value_to_gene(0.0, 16, 100.0, L),
   L == [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0].

test(3) :-
   value_to_gene(100.0, 16, 100.0, L),
   L == [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1].

test(4) :-
   gene_to_value([1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 100.0, -100.0).

test(5) :-
   gene_to_value([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 100.0, 0.0).

test(6) :-
   L = [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
   Range = 100.0,
   length(L, Width),
   gene_to_value(L, 100.0, V),
   close_enough(100.0, V, Width, Range).

test(7) :-
   Range = 100.0,
   Width = 16,
   forall((   member(V, [93.41, 87.12, 70.73, 62.84, 58.25, 43.16, 35.57, 26.98, 19.49, 5.91]),
              member(Sign, [-1, +1]),
              V1 is V * Sign),
          (   value_to_gene(V1, Width, Range, L),
              writeln([V1, L]),
              gene_to_value(L, Range, V2),
              close_enough(V1, V2, Width, Range))).


%! close_enough(+V1, +V2, +Width, +Range) is semidet.

close_enough(V1, V2, Width, Range) :-
   Quantum is Range/(1<<(Width-1)),
   abs(V1-V2) =< Quantum.

:- end_tests(spi).




