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

:- module(bisection,
          [ bisection/6
          ]).

:- meta_predicate
   bisection(2, +, +, +, +, -),
   bisection_1(2, +, +, +, +, +, +, -).

/** <module> Bisection method root finder
*/

%! bisection(+Function, +Tolerance, +Max_Iterations, +X1, +X2, -Root) is
%!           semidet.
%
%  X is a root of Function
%
%  Fails if can't find the root within Max_Iterations


bisection(F, Tolerance, Max_Iterations, X1, X2, Root) :-
   call(F, X1, Y1),
   call(F, X2, Y2),
   bisection_1(F, Tolerance, Max_Iterations, X1, Y1, X2, Y2, Root).

bisection_1(_, _, I, _, _, _, _, _) :-
   I < 0,
   !,
   fail.
bisection_1(_, Tolerance, _, X1, _, X2, _, Root) :-
   abs(X1-X2) =< Tolerance,
   !,
   Root = X1.
bisection_1(F, Tolerance, I, X1, Y1, X2, Y2, Root) :-
   X is (X1 + X2) / 2,
   call(F, X, Y),
   II is I - 1,
   (   sign(Y1) =\= sign(Y)
   ->  bisection_1(F, Tolerance, II, X1, Y1, X, Y, Root)
   ;   bisection_1(F, Tolerance, II, X, Y, X2, Y2, Root)
   ).

:- begin_tests(bisection).

test(1) :-
   bisection(fn, 0.001, 20, 0, 4.0, X),
   abs(X-1.73205) =< 0.001.

test(2) :-
   bisection(fn, 0.001, 20, -4.0, 0.0, X),
   abs(X+1.73205) =< 0.001.

fn(X, Y) :-
   Y is X*X - 3.

:- end_tests(bisection).



