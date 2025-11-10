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

