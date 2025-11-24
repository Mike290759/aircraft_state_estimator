%!  b_a(B0, B1, B2, A0, A1, A2) is det.

b_a(0.06745527, 0.13491055, 0.06745527, 1, -1.1429805, 0.4128016).

%!  sin(+Frequency, +Time, -X) is det.
%
%  @arg Frequency Hz

sin(Frequency, Time, X) :-
   X is sin(2*pi * Frequency * Time).

%! keep_top_n(+Number_To_Keep, +Number_Found, +Ranked_Population, -Top_N) is det.

:- det(keep_top_n/4).

keep_top_n(_, _, [], []) :-
   !.
keep_top_n(Number_To_Keep, N, [Individual|P], [Individual|Top_N]) :-
   N =< Number_To_Keep,
   !,
   NN is N + 1,
   keep_top_n(Number_To_Keep, NN, P, Top_N).
keep_top_n(_, _, _, []).



fitness_1(Models, Measured_OLSR_Segments, In_Points, Gene_Values, Fitness) :-
   transfer_function(Models, Gene_Values, In_Points, Out_Points),
   model_error(Out_Points, Measured_OLSR_Segments, 0, Model_Error),
   Fitness is 1 / Model_Error.


%! transfer_function(+Models, +Gene_Values:dict, +In_Points, -Out_Points) is det.

:- det(transfer_function/4).

transfer_function(Module:[model(Pred_1, Time_Step_Gene_ID_1), model(Pred_2, Time_Step_Gene_ID_2)], GV, P0, P3) :-
   transfer_function_1(Module:Pred_1, GV.Time_Step_Gene_ID_1, P0, GV, P1),
   transfer_function_1(Module:Pred_2, GV.Time_Step_Gene_ID_2, P0, GV, P2),
   resample(P1, 1, 0, X1),
   resample(P2, 1, 0, X2),
   sum_lists(X1, X2, P3).


:- det(sum_lists/3).

sum_lists([], [], []) :-
   !.
sum_lists([], _, []) :-
   !.
sum_lists(_, [], []) :-
   !.
sum_lists([p(T1, V1)|P1], [p(_, V2)|P2], [p(T1, V)|P3]) :-
   V is V1 + V2,
   !,
   sum_lists(P1, P2, P3).

%! resample(+In_Points, +Time_Step, +T, -Out_Points) is det
%
%  Out_Points is the sequence of points corresponding to Tn_Points but
%  with the specified Time_Step

resample([], _, _, []).
resample([_], _, _, []) :-
   !.
resample([p(T1, V1), p(T2, V2)|P1], Time_Step, T, [p(T, V)|P2]) :-
   T1 =< T, T < T2,
   !,
   R is (V2-V1) / (T2-T1),
   V is R * (T-T1) + V1,
   TT is T + Time_Step,
   resample([p(T1, V1), p(T2, V2)|P1], Time_Step, TT, P2).
resample([_|P1], Time_Step, T, P2) :-
   resample(P1, Time_Step, T, P2).


modelled_response(In_Points, Models, Gene_Values, Out_Points)



   points_to_segments(Model_Points, Model_Segments),
   aggregate_all(sum(Error), point_error(Measured_Points, Model_Segments, Error), Total_Error),
   Fitness is 1 / Total_Error.

%! point_error(+Measured_Points, +Model_Segments, -Error) is nondet.

point_error(Measured_Points, Model_Segments, Error) :-
   member(p(T, V_Measured), Measured_Points),
   writeln(t(T)),
   (   rb_lookup_range(T, _, V, Model_Segments)
   ->  V_Modelled = V
   ;   writeln(Measured_Points),
       throw(no_segment(T))
   ),
