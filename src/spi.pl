
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
              [ max_member/3,
                nth1/3,
                append/3,
                sum_list/2,
                nth0/3,
                min_list/2,
                max_list/2
              ]).
:- use_module(library(pce), [new/2, send/2, get/3, in_pce_thread/1]).
:- use_module(library(debug), [assertion/1]).
:- use_module(library(clpfd)).
:- use_module(library(random), [random_between/3, random_select/3, maybe/1]).
:- use_module(library(dcg/basics)).
:- use_module(src(bisection)).
:- use_module(library(aggregate), [aggregate_all/3]).
:- use_module(library(exceptions), [catch/4]).
:- use_module(library(pairs), [group_pairs_by_key/2]).

:- meta_predicate
   gen_alg_optimise(2, +, +, -),
   gen_alg_optimise_1(+, +, +, +, +, +, 2, +, +, -, -).


%:- set_prolog_flag(optimise, true).

%  M O D E L S
%
%  Second-order linear discrete transfer function filter (direct form II
%  transposed).
%
%  The general difference equation for a second-order filter is:
%
%  a0.y[n]=b0.x[n]+b1.x[n-1]+b2.x[n-2]-a1.y[n-1]-a2.y[n-2]
%
%  Rewriting the Python difference equations below gives:
%
%  y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1] + b2*x[n-2] - a2*y[n-2]
%  i.e. the same as above
%
%  Assumes A0 is normalized to 1.
%
%  B0, B1, B2 are the numerator coefficients
%  A0, A1, A2 are the denominator coefficients
%
%  For reference here is a Python implementation:
%
%  Difference equations for Direct Form II Transposed (second order)
%    y[n] = b0*x[n] + d1[n-1]
%    d1[n] = b1*x[n] - a1*y[n] + d2[n-1]
%    d2[n] = b2*x[n] - a2*y[n]
%
%    for n in range(len(x)):
%        y[n] = b[0] * x[n] + d1
%        d1 = b[1] * x[n] - a[1] * y[n] + d2
%        d2 = b[2] * x[n] - a[2] * y[n]
%
%  Reference
%  https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter.html


%! galg_config(-Dict) is det.

galg_config(galg_config{num_gene_bits: 16,
                        population_size: 500,
                        max_generations: 20,
                        r1: 0.05,              % The fittest individual is this proportion of the next generation
                        mutation_rate: 0.05}).

%! gen_x(+Step_Size, +N, -X) is nondet.

gen_x(_, _, 0).
gen_x(Step_Size, N, Step_Size) :-
    NN is N-1,
    between(1, NN, _).


%  T E S T

%!  ts is semidet.

user:ts :-
   Duration = 155,  % Duration of the measured response we want to consider
   measured_open_loop_step_response(dat('step_response.dat'), Duration, Sampling_Time_Step, Step_Size, _, Measured_OLSR),
   fitness_progress_graph(1.0, Fitness_Window, Fitness_Plot),
   RL = 1.5,
   Gene_Map = [gene(sodt_skip_interval, 1, 20), gene(b0, -RL, RL), gene(b1, -RL, RL), gene(b2, -RL, RL), gene(a1, -RL, RL), gene(a2, -RL, RL) /*,
               gene(decay_skip_interval, 1, 10), gene(d0, -RL, RL), gene(d1, -RL, RL)*/],
   length(Measured_OLSR, N),
   findall(X, gen_x(Step_Size, N, X), Step_Input_Values),
   Models = [model(sodt, b0 * x(n) + b1 * x(n-1) - a1 * y(n-1) + b2 * x(n-2) - a2 * y(n-2), sodt_skip_interval) /*,
             model(decay, d0 * x(n) + d1 * y(n-1), decay_skip_interval)*/],
   gen_alg_optimise(fitness(Measured_OLSR, Step_Input_Values, Models), Gene_Map, Fitness_Plot, Fittest_Individual),
   Fittest_Individual = individual(Best_Fitness, Fittest_Chromosome),
   chromosome_gene_values(Fittest_Chromosome, Gene_Map, Fittest_Gene_Values),
   modelled_response(Step_Input_Values, Models, Fittest_Gene_Values, Modelled_OLSR),
   plot_measured_and_modelled(Gene_Map, Fittest_Gene_Values, Best_Fitness, Duration, Sampling_Time_Step, Measured_OLSR, Modelled_OLSR),
   in_pce_thread(send(Fitness_Window, free)).


%! gene_summary(+Gene_Map, +Gene_Values:dict, -Gene_Summary:string) is
%!              det.

gene_summary(Gene_Map, Gene_Values, Gene_Summary) :-
   phrase(gene_summary_1(Gene_Map, Gene_Values), Codes),
   string_codes(Gene_Summary, Codes).


%! gene_summary_1(+Gene_Map, +Gene_Values) is det.

gene_summary_1([gene(Gene_ID, _, _)], Gene_Values) -->
   !,
   gene_summary_2(Gene_ID, Gene_Values).

gene_summary_1([gene(Gene_ID, _, _)|Gene_Map], Gene_Values) -->
   gene_summary_2(Gene_ID, Gene_Values),
   ", ",
   gene_summary_1(Gene_Map, Gene_Values).


%! gene_summary_2(+Gene_ID, +Gene_Values) is det.

gene_summary_2(Gene_ID, GV) -->
   atom(Gene_ID),
   "=",
   {format(codes(Codes), '~2f', [GV.Gene_ID])},
   Codes.


%! measured_open_loop_step_response(+File_Name, +Duration,
%!                                  -Sampling_Time_Step, -Step_Size,
%!                                  -Normalised_Step_Response) is det.
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
%  @arg Sampling_Time_Step the length of time between measurements
%  @arg Step_Size the size of input step
%  @arg N the number of data points in the step response series
%  @arg Normalisation_Factor multiply by this to convert the normalised values back to the original values
%  @arg Normalised_Step_Response list of V

measured_open_loop_step_response(File_Name, Duration, Sampling_Time_Step, Step_Size, Normalisation_Factor, Normalised_Step_Response) :-
   absolute_file_name(File_Name, Absolute_File_Name),   % Allow for path aliases
   setup_call_cleanup(open(Absolute_File_Name, read, In),
                      read_step_response(In, Sampling_Time_Step, Step_Size, L),
                      close(In)),
   rebase_time_and_trim(L, Duration, Step_Response),
   max_member(abs_max_order, Normalisation_Factor, Step_Response),
   normalised(Step_Response, Normalisation_Factor, Normalised_Step_Response).


%!  abs_max_order(+A, +B) is semidet.

abs_max_order(A, B) :-
   abs(A) =< abs(B).


%! normalised(+P, +Normalisation_Factor, -P_Normalised) is det.

normalised([], _, []).
normalised([V1|P1], Normalisation_Factor, [V2|P2]) :-
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
%
%  @arg L1 list op p(T, V)
%  @arg L2 list of values

rebase_time_and_trim(L1, Duration, L2) :-
   L1 = [p(T0, _)|_],
   T_Cutoff is T0 + Duration,
   rebase_time_and_trim_1(L1, T0, T_Cutoff, L2).


%!  rebase_time_and_trim(+L1, +T0, +T_Cutoff, +I, -L2) is det.

rebase_time_and_trim_1([], _, _, []).
rebase_time_and_trim_1([p(T, _)|_], _, T_Cutoff, []) :-
   T > T_Cutoff,
   !.
rebase_time_and_trim_1([p(_, V)|L1], T0, T_Cutoff, [V|L2]) :-
    rebase_time_and_trim_1(L1, T0, T_Cutoff, L2).


%! modelled_response(+Step_Input_Values, +I, +Models, +Gene_Values, -Model_Output_Values) is det.
%
%  Apply Models to In_Values to give Out_Values
%
%  @arg Step_Input_Values list of V
%  @arg Model_Output_Values list of V

:- det(modelled_response/4).

modelled_response(Step_Input_Values, Models, Gene_Values, Model_Output_Values) :-
    modelled_response_1(Models, Step_Input_Values, Gene_Values, Series),
    merged_series(Series, Step_Input_Values, Model_Output_Values).


%! modelled_response_1(+Models, +Step_Input_Values, +Gene_Values, -Series) is
%!                     det.
%
%  @arg Series list of model_values(Model_ID, Skip_Interval, Values)

modelled_response_1([], _, _, []).
modelled_response_1([model(Model_ID, Formula, Skip_Interval_Key)|M], Step_Input_Values, GV, [model_values(Model_ID, Skip_Interval, Values)|S]) :-
    Skip_Interval is integer(GV.Skip_Interval_Key),
    phrase(modelled_response_2(Step_Input_Values, 0, Formula, Skip_Interval, [0, 0], [0, 0], GV), Values),
    modelled_response_1(M, Step_Input_Values, GV, S).


%! modelled_response_2(+In_Values, +I, +Formula,
%!                     +Skip_Interval, +Xs, Ys, +Gene_Values) // is det.

:- det(modelled_response_2//7).

modelled_response_2([], _, _, _, _, _, _) -->
    [].
modelled_response_2([X|P], I, Formula, Skip_Interval, Xs_1, Ys_1, GV) -->
    { I mod Skip_Interval =:= 0,
      !,
      apply_formula(Formula, GV, X, Xs_1, Ys_1, Xs_2, Ys_2, Y),
      II is I + 1
    },
    [Y],
    modelled_response_2(P, II, Formula, Skip_Interval, Xs_2, Ys_2, GV).
modelled_response_2([_|P], I, Formula, Skip_Interval, Xs, Ys, GV) -->
    { II is I + 1
    },
    modelled_response_2(P, II, Formula, Skip_Interval, Xs, Ys, GV).


%!  apply_formula(+Formula, +Gene_Values, +X, +Xs, +Ys, -New_Xs, -New_Ys, -Y) is det.
%
%  In a model a formula is an expression of +, -, * operators with
%  operands:
%    * atom refers to a gene value dict id e.g. a0
%    * x(n) the current X value
%    * x(n-N) a previous X value e.g. x(n-1)
%    * y(n-N) a previous Y value e.g. y(n-2)
%
%  e.g. b0 * x(n) + b1 * x(n-1) - a1 * y(n-1) + b2 * x(n-2) - a2 * y(n-2)
%
%  Xs and Ys are the X and Y value histories. The head of the list is
%  the most recent previous value. A fixed length FIFO buffer

:- det(apply_formula/8).

apply_formula(EXPR, GV, X, Xs_0, Ys_0, Xs_1, Ys_1, Y) :-
   apply_formula_1(EXPR, GV, Xs_0, Ys_0, X, Y),
   push(X, Xs_0, Xs_1),
   push(Y, Ys_0, Ys_1).


%!  apply_formula_1(+Formula, +Gene_Values, +Xs, +Ys, +X, -Y) is det.

:- det(apply_formula_1/6).

apply_formula_1(EXPR_0, GV, Xs, Ys, X, Y) :-
   EXPR_0 =.. [Op, LHS, RHS],
   (   Op == (*)
   ;   Op == (+)
   ;   Op == (-)
   ),
   !,
   apply_formula_1(LHS, GV, Xs, Ys, X, Y_LHS),
   apply_formula_1(RHS, GV, Xs, Ys, X, Y_RHS),
   EXPR_1 =.. [Op, Y_LHS, Y_RHS],
   Y is EXPR_1.
apply_formula_1(A, GV, _, _, _, Y) :-
   atom(A),
   !,
   Y = GV.A.
apply_formula_1(x(n), _ , _, _, X, X) :-
   !.
apply_formula_1(x(n-N), _, Xs, _, _, Xn) :-
   !,
   nth1(N, Xs, Xn).
apply_formula_1(y(n-N), _, _, Ys, _, Yn) :-
   !,
   nth1(N, Ys, Yn).


%! push(+X, +Queue_In:list, -Queue_Out:list) is det.
%
%  Add X to the front of Queue_In dropping the last element

:- det(push/3).

push(X, In, Out) :-
   append(Front, [_], In),
   !,
   Out = [X|Front].


%! merged_series(+Series, +Step_Input_Values, -Out_Values) is det.
%
%  @arg Series list of model_values(Model_ID, Skip_Interval, Values)

:- det(merged_series/3).

merged_series(Series, Step_Input_Values, Out_Values) :-
    phrase(merged_series_1(Series, Step_Input_Values), Pairs),
    sort(1, @=<, Pairs, Sorted_Pairs),
    group_pairs_by_key(Sorted_Pairs, Groups),
    summed_values(Groups, Out_Values).

%!  merged_series_1(+Series, +Step_Input_Values) // is det.

merged_series_1([], _) -->
    [].
merged_series_1([model_values(_, Skip_Interval, Interval_Model_Values)|T], Step_Input_Values) -->
    gapless_model_values(Step_Input_Values, Skip_Interval, 0, _, Interval_Model_Values),
    merged_series_1(T, Step_Input_Values).


%!  gapless_model_values(+Step_Input_Values, +Skip_Interval, +I, +V, +Interval_Model_Values) // is det.

:- det(gapless_model_values//5).

gapless_model_values([], _, _, _, _) -->
    [].
gapless_model_values([_|T1], Skip_Interval, I, _, [V|T2]) -->
    { I mod Skip_Interval =:= 0,
      !,
      II is I + 1
    },
    [I-V],
    gapless_model_values(T1, Skip_Interval, II, V, T2).
gapless_model_values([_|T1], Skip_Interval, I, V, T2) -->
    { II is I + 1
    },
    [I-V],
    gapless_model_values(T1, Skip_Interval, II, V, T2).


%!  summed_values(+Groups, -Values) is det.

summed_values([], []).
summed_values([_-VL|T1], [V|T2]) :-
    sum_list(VL, V),
    summed_values(T1, T2).


:- begin_tests(merged_series).

test(1) :-
    Series = [model_values(s1, 5, [1, 2, 3, 1, 4]),
              model_values(s2, 3, [2, 3, 4, 5, 6, 7, 8, 1, 2])],
    SIV  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    merged_series(Series, SIV, OV),
    % S1 = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4],
    % S2 = [2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 1, 1, 1, 2, 2, 2],
    OV ==  [3, 3, 3, 4, 4, 5, 6, 6, 6, 7, 8, 8, 9, 9, 9, 8, 8, 8, 9, 9,12, 5, 5, 5, 6].

:- end_tests(merged_series).

:- begin_tests(formula).

test(1) :-
   X = 4,
   gv(GV),
   apply_formula(b0 * x(n) + b1 * x(n-1) - a1 * y(n-1) + b2 * x(n-2) - a2 * y(n-2), GV, X,[0,0],[0,0], Xs, Ys, Y),
   Y == 12,
   Xs == [4, 0],
   Ys == [12, 0].

test(2) :-
   X = 5,
   gv(GV),
   apply_formula(b0 * x(n) + b1 * x(n-1) - a1 * y(n-1) + b2 * x(n-2) - a2 * y(n-2), GV, X,[4,0],[12,0], Xs, Ys, Y),
   Y == 19,
   Xs == [5, 4],
   Ys == [19, 12].

test(3) :-
   X = 7,
   gv(GV),
   apply_formula(b0 * x(n) + b1 * x(n-1) - a1 * y(n-1) + b2 * x(n-2) - a2 * y(n-2), GV, X,[5,4],[19,12], Xs, Ys, Y),
   Y == 18,
   Xs == [7, 5],
   Ys == [18, 19].

gv(gene_values{a0:0, a1:1, a2:2, b0:3, b1:4, b2:5}).

:- end_tests(formula).


%! fitness(+Measured_Values, +In_Values, +Models, +Gene_Values, -Fitness) is det.
%
%  Evaluate the fitness of a set of gene values. The higher the fitness
%  the better.
%
%  See apply_formula/8 for details of the model Formula
%
%  @arg Measured_Values list of V
%  @arg In_Values list of V
%  @arg Models list of model(Model_ID, Formula, Skip_Interval_ID:atom, X_History:list, Y_History:list)
%  @arg Gene_Values dict
%  @arg Fitness float

:- det(fitness/5).

fitness(Measured_Values, In_Values, Models, GV, Fitness) :-
   modelled_response(In_Values, Models, GV, Model_Values),
   catch((aggregate_all(sum(Error), error(Measured_Values, Model_Values, Error), Total_Error),
          Fitness is 1 / Total_Error),
         evaluation_error,
         _,
         Fitness = 0).

%! error(+Measured_Values, +Model_Values, -Error) is nondet.

error([V1|_], [V2|_], Error) :-
   Error is (V1 - V2) ** 2.
error([_|T1], [_|T2], Error) :-
    error(T1, T2, Error).


%!  gen_alg_optimise(+Fitness_Pred, +Gene_Map, +Fitness_Plot,
%!                   -Fittest_Individual) is det.
%
%  @arg Gene_Map list of gene(Gene_Name, Lower_Limit, Upper_Limit)
%  @arg Fittest_Individual individual(Fitness, Chromosome)

:- det(gen_alg_optimise/4).

gen_alg_optimise(Fitness_Pred, Gene_Map, Fitness_Plot, Fittest_Individual) :-
   galg_config(C),
   initial_population(C.population_size, Gene_Map, C.num_gene_bits, Initial_Population),
   gen_alg_optimise_1(0, C.max_generations, C.mutation_rate, C.population_size, C.num_gene_bits, Gene_Map, Fitness_Pred, Fitness_Plot, Initial_Population, _, Fittest_Individual).


%! gen_alg_optimise_1(+Generation, +Max_Generations, +Mutation_Rate,
%!                    +Population_Size, +Num_Gene_Bits, +Gene_Map,
%!                    +Fitness_Pred, +Fitness_Plot, +Population_In,
%!                    -Population_Out, -Fittest_Individual) is det.
%
%  A Population is a list of individual(Fitness, Chromosome) terms where
%  Chromosome is a list of g(Gene_ID, Gene_Bits).

:- det(gen_alg_optimise_1/11).

gen_alg_optimise_1(Generation, Max_Generations, _, _, _, _, _, _, P0, P1, Fittest_Individual) :-
   Generation >= Max_Generations,
   !,
   sort(1, @>=, P0, [Fittest_Individual|_]),
   P1 = P0.
gen_alg_optimise_1(G, Max_Generations, Mutation_Rate, Population_Size, Num_Gene_Bits, Gene_Map, Fitness_Pred, Fitness_Plot, P0, P5, Fittest_Individual) :-
   assertion((length(P0, N), N == Population_Size)),
   repeat,
   swap_genes(P0, Num_Gene_Bits, P1),
   mutate(P1, Mutation_Rate, P2),
   catch(update_fitness(P2, Gene_Map, Fitness_Pred, Fitness_Plot, P3),
         evaluation_error,
         _,
         fail),
   max_member(fitness_order, individual(Best_Fitness_Of_Generation, _), P3),
   send(Fitness_Plot, append, G, Best_Fitness_Of_Generation),
   reproduce(P3, P4),
   !,  % Green cut
   GG is G + 1,
   gen_alg_optimise_1(GG, Max_Generations, Mutation_Rate, Population_Size, Num_Gene_Bits, Gene_Map, Fitness_Pred, Fitness_Plot, P4, P5, Fittest_Individual).


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


%! chromosome_gene_values(+Chromosome, +Gene_Map, -Gene_Values:dict) is det.
%
%  @arg Gene_Values dict Gene application values

:- det(chromosome_gene_values/3).

chromosome_gene_values(Chromosome, Gene_Map, Gene_Values) :-
   dict_create(GV, gene_values, []),
   chromosome_gene_values(Chromosome, Gene_Map, GV, Gene_Values).


%! chromosome_gene_values(+Chromosome, +Gene_Map, +Gene_Values:dict, -Gene_Values:dict) is det.

chromosome_gene_values([], _, GV, GV) :-
   !.
chromosome_gene_values([g(Gene_ID, Gene_Bits)|Genes], [gene(Gene_ID, Lower_Limit, Upper_Limit)|Gene_Map], GV0, GV) :-
   gene_bits_to_value(Gene_Bits, Lower_Limit, Upper_Limit, Value),
   GV1 = GV0.put([Gene_ID-Value]),
   chromosome_gene_values(Genes, Gene_Map, GV1, GV).


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


%! step_input(+Step_Size, +T, -X) is det.
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


%! fitness_progress_graph(+Y_Scale_Max, -Window, -Plot) is det.

fitness_progress_graph(Y_Scale_Max, W, Plot) :-
    new(W, auto_sized_picture('Fitness')),
    send(W, max_size, size(2000, 600)),
    send(W, display, new(Plotter, plotter)),
    galg_config(C),
    send(Plotter, axis, plot_axis(x, 0, C.max_generations, @(default), 1500)),
    send(Plotter, axis, plot_axis(y, 0, Y_Scale_Max, @(default), 400)),
    send(Plotter, graph, new(Plot, plot_graph)),
    send(Plot, colour, blue),
    send(W, open).


%! plot_measured_and_modelled(+Gene_Map, + Fittest_Gene_Values,
%                             +Best_Fitness, +Duration,
%                             +Sampling_Time_Step, +Measured_OLSR,
%                             +Modelled_OLSR) is det.
%
%  @arg Measured_OLSR list of values
%  @arg Modelled_OLSR list of values

plot_measured_and_modelled(Gene_Map, Fittest_Gene_Values, Best_Fitness, Duration, Sampling_Time_Step, Measured_OLSR, Modelled_OLSR) :-
   min_list(Measured_OLSR, Measured_Min),
   max_list(Measured_OLSR, Measured_Max),
   gene_summary(Gene_Map, Fittest_Gene_Values, Gene_Summary),
   format(string(Title), 'F=~2f, ~w', [Best_Fitness, Gene_Summary]),
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
   forall(nth0(I, Modelled_OLSR, V), (T is I * Sampling_Time_Step, send(Blue_Plot, append, T, V))),
   forall(nth0(I, Measured_OLSR, V), (T is I * Sampling_Time_Step, send(Black_Plot, append, T, V))),
   fail.
plot_measured_and_modelled(_, _, _, _, _, _, _).


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
