xmodel_error([], _, Error, Error1) => Error=Error1.
xmodel_error([p(T, _)|P], RB, E0, E) =>
   rb_lookup_range(T, _, V, RB),
   E1 is E0 + V ** 2,
   xmodel_error(P, RB, E1, E).
