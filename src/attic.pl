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


%!  points_to_segments(+Points, -RB_Tree_Out) is det.
%
%  Represent list of points as RB-tree of time ranges and values:
%  Key = T1-T2
%  Value = average of the values on point 1 and point 2

:- det(points_to_segments/2).

points_to_segments(Points, RB1) :-
   rb_empty(RB0),
   points_to_segments_1(Points, RB0, RB1).


%!  points_to_segments_1(+Points, +RB_Tree_In, -RB_Tree_Out) is det.

:- det(points_to_segments_1/3).

points_to_segments_1([], RB, RB).
points_to_segments_1([_], RB, RB) :-
   !.
points_to_segments_1([p(T1, V1), p(T2, V2)|P], RB0, RB) :-
   !,
   V is (V1 + V2) / 2,
   rb_insert(RB0, (T1-T2), V, RB1),
   points_to_segments_1([p(T2, V2)|P], RB1, RB).



%! rb_lookup_range(+Key_Value_In_Range, -Key, -Value, +RB) is
%!                 semidet.
%
%  RB is an RB-tree where each Value is associated with a key which
%  is a range Start..End. If Key_Value_In_Range is:
%
%  Start =< Key_Value_In_Range < End.
%
%  and the range exists within RB then Value is the associated value.
%
%  The =< and < range test ensures that a
%  sequence of ranges derived from a sequence of points e.g.
%
%  1, 3, 7, 11 -> 1-3, 3-7, 7-11
%
%  has any value in one range only. In the example, 3 is in the range
%  3-7 not 1-3.
%
%  @arg Key is a term Start_End

rb_lookup_range(Key_Value_In_Range, Key, Value, t(_, Tree)) =>
    rb_lookup_range_1(Key_Value_In_Range, Key, Value, Tree).

%!  rb_lookup_range_1(+Key, +Key_Range, -Value, -Tree).

rb_lookup_range_1(_, _, _, black('', _, _, '')) =>
    fail.
rb_lookup_range_1(Key, Key_Range, Value, Tree) =>
    arg(2, Tree, Start-End),
    (   Key @< Start
    ->  CMP = (<)
    ;   Start @=< Key, Key @< End
    ->  CMP = (=)
    ;   CMP = (>)
    ),
    rb_lookup_range_1(CMP, Key, Start, End, Key_Range, Value, Tree).

%!  rb_lookup_range_1(+CMP, +Key, +Start, +End, -Key_Range, -Value, -Tree).

rb_lookup_range_1(=, _, Start, End, Key_Range, Value, Tree) =>
    arg(3, Tree, Value),
    Key_Range = Start-End.
rb_lookup_range_1(<, Key, _, _, Key_Range, Value, Tree) =>
    arg(1, Tree, NTree),
    rb_lookup_range_1(Key, Key_Range, Value, NTree).
rb_lookup_range_1(>, Key, _, _, Key_Range, Value, Tree) =>
    arg(4, Tree, NTree),
    rb_lookup_range_1(Key, Key_Range, Value, NTree).


:- begin_tests(rb_lookup_range).

test(1) :-
    points_to_segments([p(0, 0), p(10, 10), p(11, 11)], RB),
    rb_lookup_range(0, KR, V, RB),
    KR == (0-10),
    V =:= 5.

test(2) :-
    points_to_segments([p(0, 0), p(10, 10), p(11, 11)], RB),
    rb_lookup_range(6, KR, V, RB),
    KR == (0-10),
    V =:= 5.

test(3) :-
    points_to_segments([p(0, 0), p(10, 10), p(11, 11)], RB),
    rb_lookup_range(10, KR, V, RB),
    KR == (10-11),
    V =:= 10.5.

test(4) :-
    points_to_segments([p(0, 0), p(10, 10), p(11, 11)], RB),
    \+ rb_lookup_range(11, _, _, RB).

:- end_tests(rb_lookup_range).


















