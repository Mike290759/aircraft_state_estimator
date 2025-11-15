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

:- use_module(library(lists),
              [ member/2,
                max_member/3,
                min_member/3
              ]).
:- use_module(library(pce), [new/2, send/2, get/3, in_pce_thread/1]).
:- use_module(library(debug), [assertion/1]).
:- use_module(library(clpfd)).
:- use_module(library(random), [random_between/3, random_select/3, maybe/1]).
:- use_module(library(exceptions), [catch/4]).
:- use_module(library(rbtrees), [list_to_rbtree/2, rb_empty/1, rb_insert/4]).
:- use_module(library(dcg/basics)).
:- use_module(src(bisection)).
:- use_module(library(aggregate), [aggregate_all/3]).

:- meta_predicate
   sodt(2, +, +, +, +, +, +, +, +, -),
   sodt_1(+, 2, +, +, +, +, +, +, +, +, +, +, -),
   galg(2, +, +, -),
   galg_1(+, +, +, +, +, +, 2, +, +, -, -),
   fitness(+, 2, +, +, -),
   fitness_1(+, 2, +, +, -).


%:- set_prolog_flag(optimise, true).

%! galg_config(-Dict) is det.

galg_config(galg_config{num_gene_bits: 16,
                        population_size: 100,
                        max_generations: 100,
                        r1: 0.15,              % Ensure the fittest individual is this proportion of the next generation
                        mutation_rate: 0.01}).


%!  ts is semidet.

user:ts :-
   Duration = 155,  % Duration of the measured response we want to consider
   measured_open_loop_step_response(dat('step_response.dat'), Duration, _, Step_Size, _, Measured_OLSR),
   fitness_progress_graph(10, Fitness_Plot),
   RL = 1.5,
   Gene_Map = [gene(sample_time, 1.0, 4.0), gene(b0, -RL, RL), gene(b1, -RL, RL), gene(b2, -RL, RL), gene(a1, -RL, RL), gene(a2, -RL, RL)],
   points_to_rb_tree(Measured_OLSR, Measured_OLSR_Segments),
   galg(fitness(Measured_OLSR_Segments, step_input(Step_Size), Duration), Gene_Map, Fitness_Plot, Fittest_Individual),
   Fittest_Individual = individual(Best_Fitness, Fittest_Chromosome),
   chromosome_gene_values(Fittest_Chromosome, Gene_Map, Fittest_Genes),
   Fittest_Genes = [Model_Sample_Time, B0, B1, B2, A1, A2],
   sodt(step_input(Step_Size), Duration, Model_Sample_Time, B0, B1, B2, 1, A1, A2, Modelled_OLSR),
   y_values(Measured_OLSR, Measured_Y_Values),
   min_member(@=<, Measured_Min, Measured_Y_Values),
   max_member(@=<, Measured_Max, Measured_Y_Values),
   gene_summary(Gene_Map, Fittest_Genes, Gene_Summary),
   format(string(Title), 'sodt - F=~2f, ~w', [Best_Fitness, Gene_Summary]),
   new(W, auto_sized_picture(Title)),
   send(W, max_size, size(2000, 600)),
   send(W, display, new(Plotter, plotter)),
   send(Plotter, axis, plot_axis(x, 0.0, Duration, @(default), 1500)),
   send(Plotter, axis, plot_axis(y, Measured_Min, Measured_Max, @(default), 400)),
   send(Plotter, graph, new(Blue_Plot, plot_graph)),
   send(Blue_Plot, colour, blue),
   send(Plotter, graph, new(Black_Plot, plot_graph)),
   send(Black_Plot, colour, black),
   legend([item('Measured Open Loop Step Response', black), item('Modelled Open Loop Step Response', blue)], Plotter, 30, 80),
   send(W, open),
   forall(member(p(T, V), Modelled_OLSR), send(Blue_Plot, append, T, V)),
   forall(member(p(T, V), Measured_OLSR), send(Black_Plot, append, T, V)),
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


%! gene_summary(+Gene_Map, +Genes, -Gene_Summary:string) is det.

gene_summary(Gene_Map, Fittest_Genes, Gene_Summary) :-
   phrase(gene_summary_1(Gene_Map, Fittest_Genes), Codes),
   string_codes(Gene_Summary, Codes).


%! gene_summary_1(+Gene_Map, +Genes) is det.

gene_summary_1([gene(Gene_ID, _, _)], [V]) -->
   !,
   gene_summary_2(Gene_ID, V).

gene_summary_1([gene(Gene_ID, _, _)|Gene_Map], [V|T]) -->
   gene_summary_2(Gene_ID, V),
   ", ",
   gene_summary_1(Gene_Map, T).


%! gene_summary_2(+Gene_ID, +V) is det.

gene_summary_2(Gene_ID, V) -->
   atom(Gene_ID),
   "=",
   {format(codes(Codes), '~2f', [V])},
   Codes.


%! measured_open_loop_step_response(+File_Name, +Duration, -Sample_Time,
%!                                  -Step_Size,
%!                                  -Normalised_Step_Response) is
%!                                  det.
%
%  Supply the data captured in the PID loop - see capture_open_loop_step_response/3
%
%  The date is time-rebased so that the first item occurs at T=0.0 and
%  the series cut off at T = Duration.
%
%  The data is also normalised so that it fits in the range -1.0..1.0
%  and the normalisation factor is returned.
%
%  @arg Duration cutoff time for the number of data points to be returned
%  @arg Sample_Time the length of time between measurements
%  @arg Step_Size the size of input step
%  @arg N the number of data points in the step response series
%  @arg Normalisation_Factor multiply by this to convert the normalised values back to the original values
%  @arg Normalised_Step_Response list of p(T, V)

measured_open_loop_step_response(File_Name, Duration, Sample_Time, Step_Size, Normalisation_Factor, Normalised_Step_Response) :-
   absolute_file_name(File_Name, Absolute_File_Name),   % Allow for path aliases
   setup_call_cleanup(open(Absolute_File_Name, read, In),
                      read_step_response(In, Sample_Time, Step_Size, L),
                      close(In)),
   rebase_time_and_trim(L, Duration, Step_Response),
   y_values(Step_Response, Y_Values),
   max_member(abs_max_order, Normalisation_Factor, Y_Values),
   normalised(Step_Response, Normalisation_Factor, Normalised_Step_Response).


%!  abs_max_order(+A, +B) is semidet.

abs_max_order(A, B) :-
   abs(A) =< abs(B).


%! normalised(+P, +Normalisation_Factor, -P_Normalised) is det.

normalised([], _, []).
normalised([p(T, V1)|P1], Normalisation_Factor, [p(T, V2)|P2]) :-
   V2 is V1 / Normalisation_Factor,
   normalised(P1, Normalisation_Factor, P2).


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


%!  sodt(+X_Pred, +Duration, +Sample_Time, +B0, +B1, +B2,
%!       +A0, +A1, +A2, -Points) is det.
%
%  Generate a time series of N inputs (X values) by passing T
%  through X_Pred and pass them through a second-order linear discrete
%  filter (direct form II transposed).
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
%  @arg Points RB-tree where key = T
%
:- det(sodt/10).

sodt(X_Pred, Duration, Sample_Time, B0, B1, B2, A0, A1, A2, P) :-
   assertion(A0 == 1),
   Time_Step = Sample_Time,
   N is integer(Duration/Sample_Time),
   D1 = 0,
   D2 = 0,
   sodt_1(N, X_Pred, 0, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, P).


%!  sodt_1(+K, +X_Pred, +Time, +Time_Step, +D1, +D2, +B0, +B1,
%!         +B2, +A0, +A1, +A2, -Points) is det.
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

:- det(sodt_1/13).

sodt_1(0, _, _, _, _, _, _, _, _, _, _, _, []) :-
   !.
sodt_1(K, X_Pred, T, Time_Step, D1, D2, B0, B1, B2, A0, A1, A2, [p(T, Y)|P]) :-
   call(X_Pred, T, X),
   Y is B0 * X + D1,
   D1_New is B1 * X - A1 * Y + D2,
   D2_New is B2 * X - A2 * Y,
   KK is K-1,
   T_Next is T + Time_Step,
   sodt_1(KK, X_Pred, T_Next, Time_Step, D1_New, D2_New, B0, B1, B2, A0, A1, A2, P).


%! fitness(+Measured_Open_Loop_Step_Response_Segments, +X_Pred,
%!         +Duration, +Gene_Values, -Fitness) is det.
%
%  Evaluate the fitness of a set gene values. The higher the fitness the
%  better.
%
%  @arg Duration the number of seconds of Measured_Open_Loop_Step_Response_RB that we are attempting to model

:- det(fitness/5).

fitness(Measured_OLSR_Segments, X_Pred, Duration, Gene_Values, Fitness) :-
   catch(fitness_1(Measured_OLSR_Segments, X_Pred, Duration, Gene_Values, Fitness),
         evaluation_error,
         _,
         Fitness = 0).


%! fitness_1(+Measured_Open_Loop_Step_Response_Segments, +X_Pred,
%!           +Duration, +Gene_Values,
%!           -Fitness) is det.

fitness_1(Measured_OLSR_Segments, X_Pred, Duration, [Sample_Time, B0, B1, B2, A1, A2], Fitness) :-
   sodt(X_Pred, Duration, Sample_Time, B0, B1, B2, 1, A1, A2, Modelled_OLSR),
   model_error(Modelled_OLSR, Measured_OLSR_Segments, 0, Model_Error),
   Fitness is 1 / Model_Error.


%! model_error(+Modelled_Open_Loop_Step_Response,
%!             +Measured_Open_Loop_Step_Response_Segments, +Fitness_Accum,
%!             -Total_Error) is det.

:- det(model_error/4).

model_error([], _, Error, Error).
model_error([p(T, V_Modelled)|P], RB, E0, E) :-
   rb_lookup_range(T, _, V_Measured, RB),
   E1 is E0 + (V_Modelled-V_Measured) ** 2,
   model_error(P, RB, E1, E).



%!  galg(+Fitness_Pred, +Gene_Map, +Fitness_Plot, -Fittest_Genes) is
%!       det.
%
%  Gene_Map list of gene(Gene_Name, Lower_Limit, Upper_Limit)

:- det(galg/4).

galg(Fitness_Pred, Gene_Map, Fitness_Plot, Fittest_Genes) :-
   galg_config(C),
   initial_population(C.population_size, Gene_Map, C.num_gene_bits, Initial_Population),
   galg_1(0, C.max_generations, C.mutation_rate, C.population_size, C.num_gene_bits, Gene_Map, Fitness_Pred, Fitness_Plot, Initial_Population, _, Fittest_Genes).


%! galg_1(+Generation, +Max_Generations, +Mutation_Rate,
%!        +Population_Size, +Num_Gene_Bits, +Gene_Map,
%!        +Fitness_Pred, +Fitness_Plot, +Population_In, -Population_Out,
%!        -Fittest_Individual) is det.
%
%  A Population is a list of individual(Fitness, Chromosome) terms where
%  Chromosome is a list of g(Gene_ID, Gene_Bits).

:- det(galg_1/11).

galg_1(Generation, Max_Generations, _, _, _, _, _, _, P0, P1, Fittest_Individual) :-
   Generation >= Max_Generations,
   !,
   sort(1, @>=, P0, [Fittest_Individual|_]),
   P1 = P0.
galg_1(G, Max_Generations, Mutation_Rate, Population_Size, Num_Gene_Bits, Gene_Map, Fitness_Pred, Fitness_Plot, P0, P5, Fittest_Individual) :-
   assertion((length(P0, N), N == Population_Size)),
   swap_genes(P0, Num_Gene_Bits, P1),
   mutate(P1, Mutation_Rate, P2),
   update_fitness(P2, Gene_Map, Fitness_Pred, Fitness_Plot, P3),
   max_member(fitness_order, individual(Best_Fitness_Of_Generation, _), P3),
   % writeln(G-f(Worst_Fitness_Of_Generation, Best_Fitness_Of_Generation)),
   in_pce_thread(send(Fitness_Plot, append, G, Best_Fitness_Of_Generation)),
   reproduce(P3, P4),
   !,  % Green cut
   GG is G + 1,
   galg_1(GG, Max_Generations, Mutation_Rate, Population_Size, Num_Gene_Bits, Gene_Map, Fitness_Pred, Fitness_Plot, P4, P5, Fittest_Individual).


%! fitness_order(+A, +B) is semidet.

fitness_order(individual(F1, _), individual(F2, _)) :-
   F1 =< F2.


%!  initial_population(+Population_Size, +Gene_Map, +Num_Gene_Bits,
%!                     -Population) is det.

:- det(initial_population/4).

initial_population(0, _, _, []) :-
   !.
initial_population(N, Gene_Map, Num_Gene_Bits, [individual(_, Chromosome)|P]) :-
   initial_chromosome(Gene_Map, Num_Gene_Bits, Chromosome),
   NN is N - 1,
   initial_population(NN, Gene_Map, Num_Gene_Bits, P).


%! initial_chromosome(+Gene_Map, +Num_Gene_Bits, -Chromosome) // is det.

initial_chromosome([], _, []).
initial_chromosome([gene(Gene_ID, _, _)|M], Num_Gene_Bits, [g(Gene_ID, Gene_Bits)|C]) :-
   initial_gene_bits(Num_Gene_Bits, Gene_Bits),
   initial_chromosome(M, Num_Gene_Bits, C).



%!  initial_gene_bits(+N, -Gene_Bits) is det.

:- det(initial_gene_bits/2).

initial_gene_bits(0, []) :-
   !.
initial_gene_bits(N, [Bit|Bits]) :-
   random_between(0, 1, Bit),
   NN is N - 1,
   initial_gene_bits(NN, Bits).


%! update_fitness(+Population, +Gene_Map, +Fitness_Pred,
%!                +Fitness_Plot,
%!                -Updated_Population) is det.

:- det(update_fitness/5).

update_fitness([], _, _, _, []).
update_fitness([individual(_, Chromosome)|P1], Gene_Map, Fitness_Pred, Fitness_Plot, [individual(Fitness, Chromosome)|P2]) :-
   chromosome_gene_values(Chromosome, Gene_Map, Gene_Values),
   call(Fitness_Pred, Gene_Values, Fitness),
   update_fitness(P1, Gene_Map, Fitness_Pred, Fitness_Plot, P2).


%!  chromosome_gene_values(+Chromosome, +Gene_Map, -Gene_Values) is det.
%
%  @arg Gene_Values list of numeric values (the application parameters)

:- det(chromosome_gene_values/3).

chromosome_gene_values([], _, []) :-
   !.
chromosome_gene_values([g(Gene_ID, Gene_Bits)|Genes], [gene(Gene_ID, Lower_Limit, Upper_Limit)|Gene_Map], [Value|Gene_Values]) :-
   gene_bits_to_value(Gene_Bits, Lower_Limit, Upper_Limit, Value),
   chromosome_gene_values(Genes, Gene_Map, Gene_Values).


%  R E P R O D U C T I O N
%
%  Order the individuals by descending fitness then
%  reproduce according to a decaying exponential function so that the
%  fittest are reproduced at a significantly greater rate than less fit
%  individuals.
%
%  If the position of an individual in the ordered list is I where I is
%  in the range [0, N-1] where N is the population size,
%  the replication function R(I) giving the multiplier for the number of
%  copies of the individual at position I to be made is:
%
%  R(I) = N * R1 * exp(-R2 * I/N)
%
%  where R1 and R2 are constants
%
%  To calculate R1, set I=0 and choose the replication factor for the
%  fittest individual, say 15%, then:
%
%  0.15 = R1 * exp(0), so R1 = 0.15
%
%  Determining R2 is a bit more complicated: the sum of R(I) over I=0..N
%  must be N so that the population size remains the same. R2 is found
%  by applying the bisection method.

%! reproduction_number(+I, +R2, -Number_Of_Clones_Required) is semidet.
%
%  N is the number of clones of the individual at fitness rank I to make
%  Fails if none are required
%
%  @arg I is the fitness rank of the individual. 0..N
%  @arg Number_Of_Clones_Required is the number of clones of individual at fitness rank I.

reproduction_number(I, R2, Number_Of_Clones_Required) :-
   galg_config(C),
   Number_Of_Clones_Required is integer(C.population_size * C.r1 * exp(-R2 * I/C.population_size)),
   Number_Of_Clones_Required > 0.


% reproduction_population_error(+R2, -Error) is det.
%
% Error is the difference between the required population and
% the number of individuals arising from the reproduction process.
% Used to calculate R2.

reproduction_population_error(R2, Error_Count) :-
   galg_config(C),
   N_Max is C.population_size-1,
   aggregate_all(sum(N),
                 (   between(0, N_Max, I),
                     reproduction_number(I, R2, N)),
                 Sigma_N),
   Error_Count is Sigma_N - C.population_size.


%! cache_r2 is det.

:- dynamic
   r2/1.

cache_r2 :-
   (   bisection(reproduction_population_error, 0.001, 50, -100, 100, R2)
   ->  retractall(r2(_)),
       assert(r2(R2))
   ;   throw(could_not_determine_r2)
   ).

:- cache_r2.


%! reproduce(+Population, -New_Population) is det.

:- det(reproduce/2).

reproduce(P0, P2) :-
   sort(1, @>=, P0, P1),
   galg_config(C),
   phrase(reproduce_1(P1, 0, C.population_size), P2).


%! reproduce_1(+Population, +I, +N) // is det.
%
%  @arg Population in reducing fitness order
%  @arg I fitness rank 0..N
%  @arg N the number still required to make up the population

reproduce_1(_, _, 0) -->
   !,
   [].
reproduce_1([individual(Fitness, Chromosome)|T], I, N) -->
   { r2(R2),
     reproduction_number(I, R2, Number_Of_Clones_Required),
     NN is N - Number_Of_Clones_Required,
     NN >= 0,
     !,
     II is I + 1
   },
   clones(Number_Of_Clones_Required, individual(Fitness, Chromosome)),
   reproduce_1(T, II, NN).
reproduce_1([Individual|T], I, N) -->
   [Individual],
   { II is I + 1,
     NN is N - 1
   },
   reproduce_1(T, II, NN).


%! clones(+Number_Of_Clones_Required, +Individual) // is det.

clones(0, _) -->
   !,
   [].
clones(N, Individual) -->
   [Individual],
   { NN is N - 1
   },
   clones(NN, Individual).



%! swap_genes(+Population, +Num_Gene_Bits, -New_Population) is det.

:- det(swap_genes/3).

swap_genes([], _, []) :-
   !.
swap_genes(P0, Num_Gene_Bits, [individual(_, C3), individual(_, C4)|P]) :-
   random_select(individual(_, C1), P0, P1),
   random_select(individual(_, C2), P1, P2),
   swap_gene_bits(Num_Gene_Bits, C1, C2, C3, C4),
   swap_genes(P2, Num_Gene_Bits, P).


%!   swap_gene_bits(+Num_Gene_Bits, +Chromosome_1, +Chromosome_2,
%!                  -Chromosome_3, -Chromosome_4) is det.

swap_gene_bits(_, [], [], [], []).
swap_gene_bits(Num_Gene_Bits, [g(Gene_ID_1, Bits_1)|C1], [g(Gene_ID_2, Bits_2)|C2], [g(Gene_ID_1, Bits_3)|C3], [g(Gene_ID_2, Bits_4)|C4]) :-
    swap_gene_bits_1(0, Num_Gene_Bits, Bits_1, Bits_2, Bits_3, Bits_4),
    swap_gene_bits(Num_Gene_Bits, C1, C2, C3, C4).


%!  swap_gene_bits_1(+Position_Index, +Swap_Position, +B1, +B2, -B3, -B4) is det.

swap_gene_bits_1(I, I, C1, C2, C1, C2) :-
   !.
swap_gene_bits_1(I, P, [H1|C1], [H2|C2], [H2|C3], [H1|C4]) :-
   II is I + 1,
   swap_gene_bits_1(II, P, C1, C2, C3, C4).


%! mutate(+Population, +Mutation_Rate, -New_Population) is det.

:- det(mutate/3).

mutate([], _, []).
mutate([individual(_, C1)|P1], Mutation_Rate, [individual(_, C2)|P2]) :-
   mutate_1(C1, Mutation_Rate, C2),
   mutate(P1, Mutation_Rate, P2).


%! mutate_1(+Genes, +Mutation_Rate, -New_Genes) is det.

:- det(mutate_1/3).

mutate_1([], _, []).
mutate_1([g(Gene_ID, Gene_Bits)|G1], Mutation_Rate, [g(Gene_ID, New_Gene_Bits)|G2]) :-
   mutate_2(Gene_Bits, Mutation_Rate, New_Gene_Bits),
   mutate_1(G1, Mutation_Rate, G2).


%! mutate_2(+Bits, +Mutation_Rate, -New_Bits) is det.

:- det(mutate_2/3).

mutate_2([], _, []).
mutate_2([B|T1], Mutation_Rate, [Not_B|T2]) :-
   maybe(Mutation_Rate),
   !,
   flip_bit(B, Not_B),
   mutate_2(T1, Mutation_Rate, T2).
mutate_2([B|T1], Mutation_Rate, [B|T2]) :-
   mutate_2(T1, Mutation_Rate, T2).


%! flip_bit(+Bit_In, -Bit_Out) is det.

flip_bit(0, 1).
flip_bit(1, 0).


%! step_input(+Step_Size, -X) is det.
%
%  Generate the step input for all time >= T=0

step_input(Step_Size, _, Step_Size).



%! gene_bits_to_value(+Bits, +Lower_Limit, +Upper_Limit, -Value) is det.
%
%  Inverse of value_to_gene_bits/4
%
%  @arg Value float
%  @arg Bits list of 1's and 0's [MSB, ... LSB]

:- det(gene_bits_to_value/4).

gene_bits_to_value(Bits, Lower_Limit, Upper_Limit, Value) :-
   length(Bits, Width),
   bits_to_int(Bits, 0, I),
   Value is Lower_Limit + ((Upper_Limit-Lower_Limit) * I/(1<<Width)).


%! bits_to_int(+Encoding, +Accum, -I) is det.

bits_to_int([], Accum, Accum).
bits_to_int([H|T], Accum, I) :-
   Accum_1 is Accum * 2 + H,
   bits_to_int(T, Accum_1, I).


:- begin_tests(spi).

test(1) :-
   gene_bits_to_value([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], -100.0, 100.0, -100.0).

test(2) :-
   gene_bits_to_value([1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], -100.0, 100.0, 0.0).

test(3) :-
   gene_bits_to_value([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], -100.0, 100.0, V),
   abs(V-100.0) =< 200.0/(1<<16).

test(4) :-
   gene_bits_to_value([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], 0.0, 5.0, V),
   abs(V-5.0) =< 5.0/(1<<16).

:- end_tests(spi).


%! fitness_progress_graph(+Y_Scale_Max, -Plot) is det.

fitness_progress_graph(Y_Scale_Max, Plot) :-
    new(W, auto_sized_picture('GALG')),
    send(W, max_size, size(2000, 600)),
    send(W, display, new(Plotter, plotter)),
    galg_config(C),
    send(Plotter, axis, plot_axis(x, 0, C.max_generations, @(default), 1500)),
    send(Plotter, axis, plot_axis(y, 0, Y_Scale_Max, @(default), 400)),
    send(Plotter, graph, new(Plot, plot_graph)),
    send(Plot, colour, blue),
    send(W, open).


%!  points_to_rb_tree(+Points, -RB_Tree_Out) is det.
%
%  Represent list of points as RB-tree of time ranges and values:
%  Key = T1-T2
%  Value = average of the values on point 1 and point 2

:- det(points_to_rb_tree/2).

points_to_rb_tree(Points, RB1) :-
   rb_empty(RB0),
   points_to_rb_tree_1(Points, RB0, RB1).


%!  points_to_rb_tree_1(+Points, +RB_Tree_In, -RB_Tree_Out) is det.

:- det(points_to_rb_tree_1/3).

points_to_rb_tree_1([], RB, RB).
points_to_rb_tree_1([_], RB, RB) :-
   !.
points_to_rb_tree_1([p(T1, V1), p(T2, V2)|P], RB0, RB) :-
   !,
   V is (V1 + V2) / 2,
   rb_insert(RB0, (T1-T2), V, RB1),
   points_to_rb_tree_1([p(T2, V2)|P], RB1, RB).



%! rb_lookup_range(+Key, +Key_Range, -Value, -RB) is semidet.
%
%  https://occasionallycogent.com/prolog_interval_tree/index.html

rb_lookup_range(Key, Key_Range, Value, t(_, Tree)) =>
    rb_lookup_range_1(Key, Key_Range, Value, Tree).

rb_lookup_range_1(_, _, _Value, black('', _, _, '')) :-
   !,
   fail.
rb_lookup_range_1(Key, Key_Range, Value, Tree) :-
    arg(2, Tree, Start-End),
    compare(CmpS, Key, Start),
    compare(CmpE, Key, End),
    rb_lookup_range_1(t(CmpS, CmpE), Key, Start-End, Key_Range, Value, Tree).

rb_lookup_range_1(t(>, <), _, Start-End, Key_Range, Value, Tree) =>
    arg(3, Tree, Value),
    Key_Range = Start-End.
rb_lookup_range_1(t(=, _), _, Start-End, Key_Range, Value, Tree) =>
    arg(3, Tree, Value),
    Key_Range = Start-End.
rb_lookup_range_1(t(_, =), _, Start-End, Key_Range, Value, Tree) =>
    arg(3, Tree, Value),
    Key_Range = Start-End.
rb_lookup_range_1(t(<, _), Key, _, Key_Range, Value, Tree) =>
    arg(1, Tree, NTree),
    rb_lookup_range_1(Key, Key_Range, Value, NTree).
rb_lookup_range_1(t(_, >), Key, _, Key_Range, Value, Tree) =>
    arg(4, Tree, NTree),
    rb_lookup_range_1(Key, Key_Range, Value, NTree).


:- begin_tests(rb_lookup_range).

test(1) :-
   L = [(0-50)-1, (51-100)-2, (101-150)-3],
   list_to_rbtree(L, RB),
   rb_lookup_range(67, KR, V, RB),
   KR == (51-100),
   V == 2.


:- end_tests(rb_lookup_range).


